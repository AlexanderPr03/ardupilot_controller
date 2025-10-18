# vehicle_control.py
from dronekit import mavutil, VehicleMode

def send_local_ned_velocity(vehicle, velocity_n, velocity_e, velocity_d):
    """
    Sends MAVLink velocity command in a world-fixed LOCAL_NED frame.
    North, East, Down.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # CRITICAL FIX: Use world-fixed frame
        0b0000111111000111, # Bitmask for velocity control
        0, 0, 0,
        velocity_n, velocity_e, velocity_d, # Velocity vectors (N, E, D)
        0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)

def land(vehicle):
    """Sets the vehicle mode to LAND."""
    print("Setting vehicle mode to LAND.")
    vehicle.mode = VehicleMode('LAND')

