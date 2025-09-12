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


class IMU:
    def __init__(self, i2c=None):
        """Initialize BNO085 IMU. If i2c is None, creates new I2C instance."""
        if i2c is None:
            self.i2c = busio.I2C(board.SCL, board.SDA)
        else:
            self.i2c = i2c
            
        # Initialize BNO085 using the working pattern from simu.py
        self.bno = BNO08X_I2C(self.i2c)
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        
        print("BNO085 IMU initialized successfully")

    def get_heading_rad(self):
        """Get heading in radians [0, 2π) from quaternion."""
        qi, qj, qk, qr = self.bno.quaternion
        # yaw from quaternion (same as soccer_robot.py)
        yaw = math.atan2(2.0 * (qr * qk + qi * qj), 1.0 - 2.0 * (qj * qj + qk * qk))
        # normalize to [0, 2π)
        if yaw < 0:
            yaw += 2.0 * math.pi
        return yaw

    def get_heading_deg(self):
        """Get heading in degrees [0, 360) from quaternion."""
        return math.degrees(self.get_heading_rad())


if __name__ == '__main__':
    # Test the IMU class
    print("Testing IMU class...")
    imu = IMU()
    
    try:
        while True:
            heading_rad = imu.get_heading_rad()
            heading_deg = imu.get_heading_deg()
            print(f"Heading: {heading_rad:.3f} rad ({heading_deg:.1f}°)")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopped")
