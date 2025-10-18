# main_controller.py
import time
from enum import Enum
import numpy as np

# --- THIS IS THE MAGIC SWITCH ---
# Set this to False when ready to connect to the real drone
SIMULATED = True

if SIMULATED:
    from mock_vehicle import connect, VehicleMode

    # In simulation, we don't have a real camera
    vision_system = None
else:
    from dronekit import connect, VehicleMode
    from vision_system import AprilTagDetector
    from vehicle_control import send_velocity_command, land


# --- Mission States ---
class MissionState(Enum):
    INITIALIZING = 1
    DROPPING = 2
    SEARCHING = 3
    HOMING = 4
    LANDING = 5
    DONE = 6


# --- Control Parameters ---
# P-controller gain for centering the tag. Tune this value carefully!
# A larger value means more aggressive corrections.
KP_GAIN = 0.005

# Forward speed when homing towards the tag (m/s)
FORWARD_SPEED = 0.5

# The area of the tag (in pixels) at which we start the landing sequence.
# This needs to be tuned based on camera resolution and desired landing height.
LANDING_AREA_THRESHOLD = 15000


def run_mission():
    """Main function to run the drone's mission state machine."""
    current_state = MissionState.INITIALIZING

    # Initialize connection to the vehicle
    connection_string = '/dev/serial0' if not SIMULATED else 'tcp:127.0.0.1:5760'
    print(f"Connecting to vehicle on: {connection_string}")
    vehicle = connect(connection_string, wait_ready=True, baud=57600)

    # Initialize vision system if not in simulation
    if not SIMULATED:
        vision = AprilTagDetector()

    try:
        while current_state != MissionState.DONE:
            print(f"\n--- Current State: {current_state.name} ---")

            if current_state == MissionState.INITIALIZING:
                vehicle.mode = VehicleMode("GUIDED_NOGPS")
                if vehicle.arm():
                    time.sleep(1)
                    current_state = MissionState.DROPPING
                else:
                    print("Arming failed. Exiting.")
                    current_state = MissionState.DONE

            elif current_state == MissionState.DROPPING:
                print("Simulating 2-second drop and stabilization...")
                time.sleep(2)
                current_state = MissionState.SEARCHING

            elif current_state == MissionState.SEARCHING:

                if SIMULATED:

                    print("SIM: Pretending to find a tag after 3 seconds.")

                    time.sleep(3)

                    current_state = MissionState.HOMING

                else:

                    frame, tag = vision.detect()

                    if tag is not None:

                        print(f"AprilTag found! ID: {tag.tag_id}. Center: {tag.center}")

                        # Stop rotating before homing

                        send_velocity_and_yaw_command(vehicle, 0, 0, 0, 0)

                        current_state = MissionState.HOMING

                    else:

                        print("No AprilTag detected. Rotating to search...")

                        # FIX: Command a slow clockwise rotation (e.g., 0.3 rad/s ~ 17 deg/s)

                        send_velocity_and_yaw_command(vehicle, 0, 0, 0, 0.3)

                        time.sleep(0.5)
            elif current_state == MissionState.HOMING:
                if SIMULATED:
                    print("SIM: Pretending to home for 5 seconds, then landing.")
                    time.sleep(5)
                    current_state = MissionState.LANDING
                else:
                    frame, tag = vision.detect()
                    if tag is None:
                        print("Tag lost! Returning to SEARCH mode.")
                        current_state = MissionState.SEARCHING
                        # Stop the drone
                        send_velocity_command(vehicle, 0, 0, 0)
                        continue

                    # Calculate tag area to decide when to land
                    corners = tag.corners
                    tag_area = 0.5 * np.abs(
                        np.dot(corners[0] - corners[2], np.cross(corners[1] - corners[3], corners[0] - corners[2])))

                    print(f"Homing on tag. Center: {tag.center}, Area: {tag_area:.2f}")

                    if tag_area > LANDING_AREA_THRESHOLD:
                        print("Tag is close enough. Proceeding to LAND.")
                        current_state = MissionState.LANDING
                        send_velocity_command(vehicle, 0, 0, 0)  # Stop movement
                    else:
                        # --- Simple Proportional Controller for Centering ---
                        # Calculate error in pixels from the center of the camera frame
                        error_x = tag.center[0] - vision.camera_center_x
                        error_y = tag.center[1] - vision.camera_center_y

                        # Calculate velocity commands (Y for left/right, Z for up/down)
                        # The signs depend on your camera orientation. This is a common setup.
                        vel_y = -KP_GAIN * error_x  # Move right for negative error (tag is left)
                        vel_z = KP_GAIN * error_y  # Move up for negative error (tag is high)

                        # Command the drone to move
                        send_velocity_command(vehicle, FORWARD_SPEED, vel_y, vel_z)
                        time.sleep(0.1)  # Loop at 10Hz

            elif current_state == MissionState.LANDING:
                if not SIMULATED:
                    land(vehicle)
                print("Landing procedure initiated.")
                current_state = MissionState.DONE

    except KeyboardInterrupt:
        print("\nMission interrupted by user.")
    finally:
        # Clean up
        if not SIMULATED and 'vision' in locals():
            vision.shutdown()

        # Stop any movement and disarm before closing
        if vehicle.armed:
            if not SIMULATED:
                send_velocity_command(vehicle, 0, 0, 0)
                vehicle.mode = VehicleMode("LAND")
            print("Disarming vehicle.")
            vehicle.armed = False  # This might not work on a real vehicle, mode change is better

        vehicle.close()
        print("\nMission finished. Resources cleaned up.")


if __name__ == '__main__':
    run_mission()
