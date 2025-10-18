# vehicle_control.py
from dronekit import mavutil, VehicleMode

# vehicle_control.py

# ... (add this new function) ...

def send_velocity_and_yaw_command(vehicle, velocity_x, velocity_y, velocity_z, yaw_rate):
    """
    Sends a velocity command that also includes a yaw rate.
    `yaw_rate` is in radians/second (positive is clockwise).
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111 & ~0b0000010000000000, # Same as before, but un-sets the 'ignore yaw rate' bit
        0, 0, 0,
        velocity_x, velocity_y, velocity_z,
        0, 0, 0,
        0, yaw_rate) # Use yaw_rate field
    vehicle.send_mavlink(msg)

def land(vehicle):
    """Sets the vehicle mode to LAND."""
    print("Setting vehicle mode to LAND.")
    # FIX: Use the VehicleMode object, not a string
    vehicle.mode = VehicleMode('LAND')