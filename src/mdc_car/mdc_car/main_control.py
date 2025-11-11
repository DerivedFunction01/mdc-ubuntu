import math
import rclpy
from rclpy.node import Node
import contextlib
from geometry_msgs.msg import Twist
from .include.maestro import Maestro
from typing import Iterator

# Config
_MAESTRO_PATH = "/dev/ttyACM0"
_MAESTRO_BAUD = 9600
_CHANNEL_STEER = 1
_CHANNEL_DRIVE = 0
_NEUTRAL_DRIVE = 6000
_BACK_MIN = 5640
_FWD_MIN = 6200
_NEUTRAL_STEER = 5800
_TRIM = 50
_STEER_LEFT = 4200
_STEER_RIGHT = 7600
_LEFT_RANGE = _NEUTRAL_STEER - _STEER_LEFT
_RIGHT_RANGE = _STEER_RIGHT - _NEUTRAL_STEER
_WHEELBASE = 0.3556
_LIMIT = 0.25
_ACCEL = 0
TOPIC = '/cmd_vel'

# Linear fits
_M_FWD = 0.002306997333
_B_FWD = -0.4474072417
_TOP_FWD = int((_LIMIT - _B_FWD) / _M_FWD + _NEUTRAL_DRIVE)
_M_REV = 0.001061176966
_B_REV = -0.3531598483
_TOP_REV = int(_NEUTRAL_DRIVE - (_LIMIT - _B_REV) / _M_REV)

_M_R = 0.01269313305
_B_R = -0.3433476395
_M_L = 0.0169201995
_B_L = -4.554862843

class MainControl(Node):
    def __init__(self, mdev):
        super().__init__('main_control')
        self.mdev = mdev
        self.sub = self.create_subscription(Twist, TOPIC, self.cb, 10)

    def cb(self, msg):
        # Drive
        if msg.linear.x > 0:
            drive = int(msg.linear.x * (_TOP_FWD - _FWD_MIN) + _FWD_MIN)
        elif msg.linear.x < 0:
            drive = int(msg.linear.x * (_BACK_MIN - _TOP_REV) + _BACK_MIN)
        else:
            drive = _NEUTRAL_DRIVE
        drive = max(_TOP_REV, min(_TOP_FWD, drive))
        drive_off = abs(drive - _NEUTRAL_DRIVE)

        # Steer
        steer_range = _LEFT_RANGE if msg.angular.z else 0
        steer = int(_NEUTRAL_STEER - msg.angular.z * steer_range + _TRIM)
        steer = max(_STEER_LEFT, min(_STEER_RIGHT, steer))
        steer_off = abs(steer - _NEUTRAL_STEER)

        # Angle
        if steer < _NEUTRAL_STEER:
            angle = -(_M_L * steer_off + _B_L)
        elif steer > _NEUTRAL_STEER:
            angle = _M_R * steer_off + _B_R
        else:
            angle = 0.0

        # Speed
        if drive >= _FWD_MIN:
            speed = _M_FWD * drive_off + _B_FWD
        elif drive <= _BACK_MIN:
            speed = _M_REV * drive_off + _B_REV
        else:
            speed = 0.0

        radius = calc_radius(speed, angle)
        
        try:
            self.mdev.set_target(_CHANNEL_DRIVE, drive)
            self.mdev.set_target(_CHANNEL_STEER, steer) # steer is in degrees
            self.get_logger().info(f'D:{drive} S:{speed:.2f}m/s A:{angle:.1f}° R:{radius:.1f}m')
        except Exception as e:
            self.get_logger().error(f'Err: {e}')

@contextlib.contextmanager
def maestro() -> Iterator[Maestro]:
    m = Maestro(_MAESTRO_PATH, _MAESTRO_BAUD)
    try:
        yield m
    finally:
        m.close()

def calc_radius(speed, deg, mu=1.0):
    """Calculates the radius of a wheel based on speed and angle.

    Args:
        speed (float): The speed of the vehicle in meters per second.
        deg (float): The steering angle of the vehicle in degrees.
        mu (float, optional): The coefficient of friction between the tires and the ground. Defaults to 1.0.

    Returns:
        float: The radius of a wheel based on speed and angle.
    """
    if abs(deg) < 0.01 or speed == 0:
        return float('inf')
    rad = math.radians(deg)
    static = abs(_WHEELBASE / math.tan(rad))
    lat = speed**2 / static
    if lat > mu*9.81:
        return speed**2 / (mu*9.81)
    return static

def main():
    global _TOP_FWD, _TOP_REV, _ACCEL
    rclpy.init()
    # Config (defaults)
    _LIMIT = 0.25
    _ACCEL = 0
    
    # Interactive input with defaults
    speed_str = input(f"Top speed (m/s) [{_LIMIT}]: ") or str(_LIMIT)
    try: _LIMIT = float(speed_str); assert _LIMIT > 0
    except: print("Invalid, using default"); _LIMIT = 0.25
    
    accel_str = input(f"Acceleration (0-255) [{_ACCEL}]: ") or str(_ACCEL)
    try: _ACCEL = int(accel_str); assert 0 <= _ACCEL <= 255
    except: print("Invalid, using default"); _ACCEL = 0
    
    # Recalculate limits
    _TOP_FWD = int((_LIMIT - _B_FWD) / _M_FWD + _NEUTRAL_DRIVE)
    _TOP_REV = int(_NEUTRAL_DRIVE - (_LIMIT - _B_REV) / _M_REV)
    
    with maestro() as m:
        m.set_target(_CHANNEL_STEER, _NEUTRAL_STEER)
        m.set_speed(_CHANNEL_STEER, 0)
        m.set_acceleration(_CHANNEL_STEER, 0)
        m.set_target(_CHANNEL_DRIVE, _NEUTRAL_DRIVE)
        m.set_speed(_CHANNEL_DRIVE, _ACCEL)
        m.set_acceleration(_CHANNEL_DRIVE, 0)
        node = MainControl(m)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
