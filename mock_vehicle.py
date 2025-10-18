# mock_vehicle.py
import time
import math

class MockAttitude:
    def __init__(self): self.pitch, self.yaw, self.roll = 0.0, 0.0, 0.0
    def __str__(self): return f"Attitude(pitch={self.pitch}, yaw={self.yaw}, roll={self.roll})"

class MockLocationLocal:
    def __init__(self): self.north, self.east, self.down = 0.0, 0.0, -75.0
    def __str__(self): return f"Local(N={self.north}, E={self.east}, D={self.down})"

class MockLocation:
    def __init__(self): self.local_frame = MockLocationLocal()
    def __str__(self): return f"Location(local={self.local_frame})"

class MockAccel:
    def __init__(self): self.x, self.y, self.z = 0.0, 0.0, -9.81
    def __str__(self): return f"Acceleration(x={self.x}, y={self.y}, z={self.z})"

class MockVehicle:
    def __init__(self, connection_string, baud=None, wait_ready=True):
        print(f"SIMULATED: Connecting to vehicle on {connection_string}...")
        self.armed = False
        self.mode = 'STABILIZE'
        self.location = MockLocation()
        self.attitude = MockAttitude()
        self.acceleration = MockAccel()
        self.is_armable = True
        self.message_factory = self._message_factory()
        print("SIMULATED: Connection successful!")

    def send_mavlink(self, message): pass

    def arm(self):
        if self.is_armable: self.armed = True; print("SIMULATED: Vehicle ARMED."); return True
        return False

    def close(self): print("SIMULATED: Closing connection.")

    def _message_factory(self):
        class Encoder:
            def set_position_target_local_ned_encode(*args, **kwargs): return "Mocked MAVLink"
        return Encoder()

    @property
    def mode(self): return self._mode

    @mode.setter
    def mode(self, new_mode):
        self._mode = new_mode if isinstance(new_mode, str) else new_mode.name
        print(f"SIMULATED: Vehicle mode set to {self._mode}")

def connect(connection_string, wait_ready=True, baud=None): return MockVehicle(connection_string)

class VehicleMode:
    def __init__(self, mode_name): self.name = mode_name

