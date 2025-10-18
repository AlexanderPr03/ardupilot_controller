# failsafe_land_controller.py
# A simplified mission script that serves as a failsafe or baseline test.
# Mission Profile: Wait -> Drop -> Stabilize -> Immediately Land.
# This script does NOT use the camera or any vision-based navigation.

import time
from enum import Enum

# --- THIS IS THE MAGIC SWITCH ---
SIMULATED = True

# --- Mission Parameters ---
PRE_FLIGHT_WAIT_S = 120  # 2-minute wait time after power-on
DROP_DURATION_S = 1  # Time motors are off after pre-flight wait
STABILIZE_TIME_S = 5  # Seconds to hover and stabilize before landing

# --- Imports ---
if SIMULATED:
    from mock_vehicle import connect, VehicleMode
else:
    from dronekit import connect, VehicleMode
    from vehicle_control import send_velocity_command, land


# --- Mission States ---
class MissionState(Enum):
    PRE_FLIGHT_DELAY = 0
    DROPPING = 1
    ARMING_MOTORS = 2
    STABILIZING = 3
    LANDING = 4
    DONE = 5
    MISSION_FAILED = 6


def run_failsafe_mission():
    """Main function to run the drone's simple land mission."""
    current_state = MissionState.PRE_FLIGHT_DELAY

    connection_string = '/dev/serial0' if not SIMULATED else 'tcp:127.0.0.1:5760'

    vehicle = None  # Initialize vehicle to None
    try:
        print("Connecting to vehicle...")
        vehicle = connect(connection_string, wait_ready=True, baud=57600)
        print("Connection successful.")

        while current_state not in [MissionState.DONE, MissionState.MISSION_FAILED]:
            print(f"\n--- State: {current_state.name} ---")

            if current_state == MissionState.PRE_FLIGHT_DELAY:
                print(f"In pre-flight hold. Waiting for {PRE_FLIGHT_WAIT_S} seconds.")
                time.sleep(PRE_FLIGHT_WAIT_S)
                current_state = MissionState.DROPPING

            elif current_state == MissionState.DROPPING:
                print(f"Simulating {DROP_DURATION_S}s freefall (motors off)...")
                time.sleep(DROP_DURATION_S)
                current_state = MissionState.ARMING_MOTORS

            elif current_state == MissionState.ARMING_MOTORS:
                print("Setting mode to GUIDED_NOGPS and attempting to arm...")
                if not SIMULATED:
                    vehicle.mode = VehicleMode("GUIDED_NOGPS")
                    if vehicle.arm():
                        print("Vehicle armed successfully.")
                        current_state = MissionState.STABILIZING
                    else:
                        print("CRITICAL: Arming failed.")
                        current_state = MissionState.MISSION_FAILED
                else:  # Simulation
                    vehicle.arm()
                    current_state = MissionState.STABILIZING

            elif current_state == MissionState.STABILIZING:
                print(f"Stabilizing hover for {STABILIZE_TIME_S} seconds...")
                if not SIMULATED:
                    # Command the drone to stop all movement
                    send_velocity_command(vehicle, 0, 0, 0)
                time.sleep(STABILIZE_TIME_S)
                current_state = MissionState.LANDING

            elif current_state == MissionState.LANDING:
                print("Vision-less landing procedure initiated.")
                if not SIMULATED:
                    land(vehicle)
                current_state = MissionState.DONE

            time.sleep(0.1)

    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
        current_state = MissionState.MISSION_FAILED
    finally:
        if vehicle:
            # As a final safety measure, if the script ends and the vehicle is still armed, command it to land.
            if vehicle.armed and not SIMULATED:
                print("EMERGENCY: Script ending while armed. Sending final land command.")
                land(vehicle)
            vehicle.close()

        print(f"\nMission ended in state: {current_state.name}.")


if __name__ == '__main__':
    run_failsafe_mission()
