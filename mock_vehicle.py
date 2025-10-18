# mock_vehicle.py
import time
import math

class MockAttitude:
    def __init__(self):
        self.pitch = 0.0
        self.yaw = math.radians(90) # Start facing East
        self.roll = 0.0
    def __str__(self):
        return f"Attitude(pitch={self.pitch}, yaw={self.yaw}, roll={self.roll})"

class MockLocation:
    def __init__(self, alt):
        self.alt = alt
    def __str__(self):
        return f"Location(alt={self.alt})"

class MockVehicle:
    """A mock vehicle class that mimics DroneKit's Vehicle class for simulation."""
    def __init__(self, connection_string, baud=None, wait_ready=True):
        print(f"SIMULATED: Connecting to vehicle on {connection_string}...")
        self.armed = False
        self.mode = 'STABILIZE'
        self.location = MockLocation(alt=20.0) # Start at a "dropped" altitude
        self.attitude = MockAttitude()
        self.is_armable = True
        self.message_factory = self._message_factory()
        print("SIMULATED: Connection successful!")

    def send_mavlink(self, message):
        """Simulates sending a MAVLink message."""
        print(f"SIMULATED: Received MAVLink Message: {message}")

    def arm(self):
        print("SIMULATED: Arming motors...")
        if self.is_armable:
            self.armed = True
            print("SIMULATED: Vehicle ARMED.")
            return True
        print("SIMULATED: Arming failed.")
        return False

    def close(self):
        print("SIMULATED: Closing connection.")

    def _message_factory(self):
        """A dummy message factory to avoid errors."""
        class Encoder:
            def set_position_target_local_ned_encode(*args, **kwargs):
                return "Mocked MAVLink Position Message"
        return Encoder()

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, new_mode):
        self._mode = new_mode if isinstance(new_mode, str) else new_mode.name
        print(f"SIMULATED: Vehicle mode set to {self._mode}")


# This is a mock connect function to mimic `dronekit.connect`
def connect(connection_string, wait_ready=True, baud=None):
    return MockVehicle(connection_string, wait_ready=wait_ready, baud=baud)

# Mock VehicleMode for compatibility
class VehicleMode:
    def __init__(self, mode_name):
        self.name = mode_name
