# main_controller.py
import time
from enum import Enum
import numpy as np
import math

# --- THIS IS THE MAGIC SWITCH ---
SIMULATED = True

# --- Vision System Choice ---
# Set to 'APRILTAG' or 'PAPER'
VISION_MODE = 'APRILTAG'

# --- Mission & Control Parameters ---
TARGET_TAG_ID = 7
PRE_FLIGHT_WAIT_S = 120
DROP_DURATION_S = 1
STABILIZE_TIME_S = 3
TARGET_LOST_HOVER_S = 2  # Time to hover if tag is lost during homing

# --- Safety Limit Parameters ---
MISSION_TIMEOUT_S = 240
MAX_SPEED_MS = 5.56
GEOFENCE_RADIUS_M = 100

# --- Search Algorithm Parameters ---
SEARCH_ALTITUDE = -45.0  # Target altitude for search (negative is UP in NED)
SEARCH_SPEED_MS = 2.0
SEARCH_LEG_LENGTH_M = 20
SEARCH_LANE_WIDTH_M = 10
MAX_SEARCH_LEGS = 10

# --- Homing & Safety Parameters ---
HOMING_DESCENT_SPEED_MS = 1.0  # Use positive for DOWN in NED
KP_GAIN_XY = 0.005
KP_GAIN_Z = 0.003
MAX_ACCELERATION_G = 2.0

# --- Landing Parameters ---
LANDING_AREA_THRESHOLD = 25000

# --- Imports ---
if SIMULATED:
    from mock_vehicle import connect, VehicleMode

    vision = None
else:
    from dronekit import connect, VehicleMode

    if VISION_MODE == 'APRILTAG':
        from vision_system import AprilTagDetector as VisionSystem
    else:
        from paper_detector import PaperDetector as VisionSystem
    from vehicle_control import send_local_ned_velocity, land


# --- Enums for States ---
class MissionState(Enum):
    PRE_FLIGHT_DELAY = 0;
    DROPPING = 1;
    ARMING_MOTORS = 2;
    STABILIZING = 3
    SEARCHING = 4;
    HOMING = 5;
    LANDING = 6;
    MISSION_FAILED = 7;
    DONE = 8


class SearchSubState(Enum):
    PLANNING_LEG = 1;
    EXECUTING_LEG = 2;
    PLANNING_TURN = 3;
    EXECUTING_TURN = 4


def run_mission():
    current_state = MissionState.PRE_FLIGHT_DELAY
    mission_start_time = 0
    mission_timer_started = False
    search_sub_state = SearchSubState.PLANNING_LEG
    search_leg_count = 0
    target_position = np.array([0.0, 0.0, SEARCH_ALTITUDE])

    connection_string = '/dev/serial0' if not SIMULATED else 'tcp:127.0.0.1:5760'
    vehicle = connect(connection_string, wait_ready=True, baud=57600)

    if not SIMULATED: vision = VisionSystem()

    try:
        while current_state not in [MissionState.DONE, MissionState.MISSION_FAILED]:
            if mission_timer_started and (time.time() - mission_start_time > MISSION_TIMEOUT_S):
                print(f"TIMEOUT! Exceeded {MISSION_TIMEOUT_S}s. Aborting.");
                current_state = MissionState.MISSION_FAILED;
                continue

            current_position = np.array([
                vehicle.location.local_frame.north, vehicle.location.local_frame.east, vehicle.location.local_frame.down
            ]) if not SIMULATED else target_position  # In sim, just assume we are at the target

            if np.linalg.norm(current_position[:2]) > GEOFENCE_RADIUS_M:
                print(f"GEOFENCE BREACHED! Pos: {current_position}. Aborting.");
                current_state = MissionState.MISSION_FAILED;
                continue

            print(
                f"\n--- State: {current_state.name} | Pos: ({current_position[0]:.1f}N, {current_position[1]:.1f}E, {current_position[2]:.1f}D)m ---")

            if current_state == MissionState.PRE_FLIGHT_DELAY:
                time.sleep(PRE_FLIGHT_WAIT_S);
                current_state = MissionState.DROPPING
            elif current_state == MissionState.DROPPING:
                time.sleep(DROP_DURATION_S);
                current_state = MissionState.ARMING_MOTORS
            elif current_state == MissionState.ARMING_MOTORS:
                vehicle.mode = VehicleMode("GUIDED_NOGPS")
                current_state = MissionState.STABILIZING if vehicle.arm() else MissionState.MISSION_FAILED
            elif current_state == MissionState.STABILIZING:
                if not SIMULATED: send_local_ned_velocity(vehicle, 0, 0, 0)
                time.sleep(STABILIZE_TIME_S)
                if not mission_timer_started: mission_start_time = time.time(); mission_timer_started = True
                current_state = MissionState.SEARCHING

            elif current_state == MissionState.SEARCHING:
                if SIMULATED: time.sleep(2); current_state = MissionState.HOMING; continue

                if VISION_MODE == 'APRILTAG':
                    frame, tag = vision.detect(target_tag_id=TARGET_TAG_ID)
                else:
                    frame, tag = vision.detect()

                if tag: print("Target found!"); current_state = MissionState.HOMING; continue
                if search_leg_count > MAX_SEARCH_LEGS: print(
                    "Max search legs reached."); current_state = MissionState.MISSION_FAILED; continue

                if search_sub_state == SearchSubState.PLANNING_LEG:
                    direction = 1 if search_leg_count % 4 < 2 else -1  # Fly North, then South
                    axis = 0 if search_leg_count % 2 == 0 else 1  # Alternate North/South and East/West legs
                    if axis == 0:
                        target_position = current_position + np.array([SEARCH_LEG_LENGTH_M * direction, 0, 0])
                    else:
                        target_position = current_position + np.array([0, SEARCH_LEG_LENGTH_M * direction, 0])
                    search_sub_state = SearchSubState.EXECUTING_LEG
                elif search_sub_state == SearchSubState.EXECUTING_LEG:
                    error_to_target = target_position - current_position
                    if np.linalg.norm(error_to_target) < 2.0:
                        search_sub_state = SearchSubState.PLANNING_TURN
                    else:
                        direction_vector = (target_position - current_position) / np.linalg.norm(error_to_target)
                        velocity = direction_vector * SEARCH_SPEED_MS
                        send_local_ned_velocity(vehicle, velocity[0], velocity[1], 0)  # Hold altitude
                elif search_sub_state == SearchSubState.PLANNING_TURN:
                    # Plan to shift to the next lane
                    axis = 1 if search_leg_count % 2 == 0 else 0  # Turn East/West or North/South
                    target_position = current_position + np.array(
                        [0 if axis == 1 else SEARCH_LANE_WIDTH_M, SEARCH_LANE_WIDTH_M if axis == 1 else 0, 0])
                    search_sub_state = SearchSubState.EXECUTING_TURN
                elif search_sub_state == SearchSubState.EXECUTING_TURN:
                    error_to_target = target_position - current_position
                    if np.linalg.norm(error_to_target) < 2.0:
                        search_leg_count += 1
                        search_sub_state = SearchSubState.PLANNING_LEG
                    else:
                        direction_vector = (target_position - current_position) / np.linalg.norm(error_to_target)
                        velocity = direction_vector * SEARCH_SPEED_MS
                        send_local_ned_velocity(vehicle, velocity[0], velocity[1], 0)

            elif current_state == MissionState.HOMING:
                if SIMULATED: time.sleep(3); current_state = MissionState.LANDING; continue

                if VISION_MODE == 'APRILTAG':
                    frame, tag = vision.detect(target_tag_id=TARGET_TAG_ID)
                else:
                    frame, tag = vision.detect()

                if tag is None:
                    print("Target lost! Hovering for recovery...");
                    send_local_ned_velocity(vehicle, 0, 0, 0)
                    time.sleep(TARGET_LOST_HOVER_S)
                    current_state = MissionState.SEARCHING
                    continue

                if tag.area > LANDING_AREA_THRESHOLD: print(
                    "Target is close, beginning landing."); current_state = MissionState.LANDING; continue

                error_x = tag.center[0] - vision.camera_center_x
                error_y = tag.center[1] - vision.camera_center_y

                # Corrected Controller Logic for LOCAL_NED frame
                vel_n = KP_GAIN_Z * error_y  # Forward/backward (North)
                vel_e = -KP_GAIN_XY * error_x  # Left/right (East)
                vel_d = HOMING_DESCENT_SPEED_MS  # Constant descent (Down)

                send_local_ned_velocity(vehicle, vel_n, vel_e, vel_d)

            elif current_state == MissionState.LANDING:
                if not SIMULATED: land(vehicle)
                current_state = MissionState.DONE

            time.sleep(0.1)

    except Exception as e:
        print(f"An unhandled exception occurred: {e}");
        current_state = MissionState.MISSION_FAILED
    finally:
        if vehicle:
            if vehicle.armed and not SIMULATED: land(vehicle)
            vehicle.close()
        if not SIMULATED and vision: vision.shutdown()
        print(f"\nMission ended in state: {current_state.name}.")


if __name__ == '__main__':
    run_mission()

