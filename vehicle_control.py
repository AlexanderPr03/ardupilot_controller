# vehicle_control.py
from dronekit import mavutil, VehicleMode

def send_velocity_command(vehicle, velocity_x, velocity_y, velocity_z):
    """
    Sends MAVLink SET_POSITION_TARGET_LOCAL_NED message to control velocity.
    `velocity_x` is forward (positive) in m/s.
    `velocity_y` is right (positive) in m/s.
    `velocity_z` is down (positive) in m/s.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111, # Bitmask for velocity control
        0, 0, 0,
        velocity_x, velocity_y, velocity_z,
        0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)

def send_velocity_and_yaw_command(vehicle, velocity_x, velocity_y, velocity_z, yaw_rate):
    """
    Sends a velocity command that also includes a yaw rate for searching.
    `yaw_rate` is in radians/second (positive is clockwise).
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        # CORRECT: Un-set the 'ignore yaw rate' bit to enable rotation
        0b0000111111000111 & ~0b0000010000000000,
        0, 0, 0,
        velocity_x, velocity_y, velocity_z,
        0, 0, 0,
        0, yaw_rate)
    vehicle.send_mavlink(msg)

def land(vehicle):
    """Sets the vehicle mode to LAND."""
    print("Setting vehicle mode to LAND.")
    # CORRECT: Use the VehicleMode object, not a string
    vehicle.mode = VehicleMode('LAND')

