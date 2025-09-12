import time
import math
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO08X_I2C(i2c)
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

def get_heading_rad():
    """Get heading in radians [0, 2π) from quaternion."""
    qi, qj, qk, qr = bno.quaternion
    # yaw from quaternion (same as soccer_robot.py)
    yaw = math.atan2(2.0 * (qr * qk + qi * qj), 1.0 - 2.0 * (qj * qj + qk * qk))
    # normalize to [0, 2π)
    if yaw < 0:
        yaw += 2.0 * math.pi
    return yaw

print("BNO085 initialized. Reading heading...")
print("Press Ctrl+C to stop")

try:
    while True:
        heading_rad = get_heading_rad()
        heading_deg = math.degrees(heading_rad)
        print(f"Heading: {heading_rad:.3f} rad ({heading_deg:.1f}°)")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nStopped")