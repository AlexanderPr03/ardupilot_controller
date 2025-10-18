import time
from enum import Enum
import numpy as np

# --- THIS IS THE MAGIC SWITCH ---
SIMULATED = True

# --- Mission & Control Parameters (TUNE THESE CAREFULLY!) ---
TARGET_TAG_ID = 7  # The specific AprilTag ID the drone should look for
STABILIZE_TIME_S = 3  # Seconds to hover and stabilize after the drop
SEARCH_YAW_RATE_RPS = 0.3  # Rotation speed in radians/sec during search (~17 deg/s)

# --- Homing Parameters ---
FORWARD_SPEED_MS = 0.7  # Forward speed towards the tag in m/s
DESCENT_SPEED_MS = 0.5  # Downward speed during diagonal approach in m/s
KP_GAIN_XY = 0.004  # P-controller gain for sideways correction. START LOW!

# --- Landing Parameters ---
LANDING_AREA_THRESHOLD = 20000  # Pixel area of the tag to trigger the final landing.

# --- Imports ---
if SIMULATED:
    from mock_vehicle import connect, VehicleMode

    vision = None
else:
    from dronekit import connect, VehicleMode
    from vision_system import AprilTagDetector
    # CORRECT: Import all necessary control functions
    from vehicle_control import send_velocity_command, land, send_velocity_and_yaw_command


# --- Mission States ---
class MissionState(Enum):
    INITIALIZING = 1
    DROPPING = 2
    STABILIZING = 3  # NEW: State to recover from the drop
    SEARCHING = 4
    HOMING = 5
    LANDING = 6
    DONE = 7


def run_mission():
    """Main function to run the drone's mission state machine."""
    current_state = MissionState.INITIALIZING
    connection_string = '/dev/serial0' if not SIMULATED else 'tcp:127.0.0.1:5760'
    vehicle = connect(connection_string, wait_ready=True, baud=57600)

    if not SIMULATED:
        vision = AprilTagDetector()

    try:
        while current_state != MissionState.DONE:
            print(f"\n--- Current State: {current_state.name} ---")

            if current_state == MissionState.INITIALIZING:
                vehicle.mode = VehicleMode("GUIDED_NOGPS")
                if vehicle.arm():
                    current_state = MissionState.DROPPING
                else:
                    print("Arming failed. Exiting.")
                    current_state = MissionState.DONE

            elif current_state == MissionState.DROPPING:
                print("Simulating 1-second freefall...")
                time.sleep(1)
                current_state = MissionState.STABILIZING

            elif current_state == MissionState.STABILIZING:
                print(f"Stabilizing for {STABILIZE_TIME_S} seconds...")
                if not SIMULATED:
                    # CORRECT: Command a hover to achieve level flight
                    send_velocity_command(vehicle, 0, 0, 0)
                time.sleep(STABILIZE_TIME_S)
                current_state = MissionState.SEARCHING

            elif current_state == MissionState.SEARCHING:
                if SIMULATED:
                    print(f"SIM: Pretending to find tag ID {TARGET_TAG_ID}...")
                    time.sleep(2)
                    current_state = MissionState.HOMING
                else:
                    # CORRECT: Pass the specific tag ID to the vision system
                    frame, tag = vision.detect(target_tag_id=TARGET_TAG_ID)
                    if tag is not None:
                        print(f"Target AprilTag #{tag.tag_id} found!")
                        send_velocity_and_yaw_command(vehicle, 0, 0, 0, 0)  # Stop rotating
                        current_state = MissionState.HOMING
                    else:
                        print("No target tag detected. Rotating to search...")
                        # CORRECT: Command a slow rotation to scan the area
                        send_velocity_and_yaw_command(vehicle, 0, 0, 0, SEARCH_YAW_RATE_RPS)
                        time.sleep(0.5)

            elif current_state == MissionState.HOMING:
                if SIMULATED:
                    print("SIM: Pretending to home for 5s, then landing.")
                    time.sleep(5)
                    current_state = MissionState.LANDING
                else:
                    frame, tag = vision.detect(target_tag_id=TARGET_TAG_ID)
                    if tag is None:
                        print("Tag lost! Returning to SEARCH mode.")
                        send_velocity_command(vehicle, 0, 0, 0)  # Stop and hover
                        time.sleep(1)
                        current_state = MissionState.SEARCHING
                        continue

                    corners = tag.corners
                    tag_area = 0.5 * np.abs(
                        np.dot(corners[0] - corners[2], np.cross(corners[1] - corners[3], corners[0] - corners[2])))
                    print(f"Homing on tag. Area: {tag_area:.2f}")

                    if tag_area > LANDING_AREA_THRESHOLD:
                        print("Tag is close. Proceeding to LAND.")
                        send_velocity_command(vehicle, 0, 0, 0)
                        current_state = MissionState.LANDING
                    else:
                        error_x = tag.center[0] - vision.camera_center_x
                        vel_y = -KP_GAIN_XY * error_x

                        # CORRECT: Command forward and constant downward speed for a true diagonal approach
                        print(f"ErrorX: {error_x:.2f}, VelY: {vel_y:.2f}. Approaching diagonally.")
                        send_velocity_command(vehicle, FORWARD_SPEED_MS, vel_y, DESCENT_SPEED_MS)
                        time.sleep(0.1)

            elif current_state == MissionState.LANDING:
                if not SIMULATED:
                    land(vehicle)
                print("Landing procedure initiated.")
                current_state = MissionState.DONE

    except KeyboardInterrupt:
        print("\nMission interrupted by user.")
    finally:
        if vehicle and vehicle.armed and not SIMULATED:
            print("EMERGENCY: Sending stop and land command.")
            send_velocity_command(vehicle, 0, 0, 0)
            land(vehicle)
        if vehicle:
            vehicle.close()
        if not SIMULATED and vision:
            vision.shutdown()
        print("\nMission finished. Resources cleaned up.")


if __name__ == '__main__':
    run_mission()