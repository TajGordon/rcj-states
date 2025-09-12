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


class RealIMU:
    def __init__(self, i2c=None):
        """Initialize BNO085 IMU with corrected counter-clockwise angle handling.
        
        If i2c is None, creates new I2C instance.
        """
        if i2c is None:
            self.i2c = busio.I2C(board.SCL, board.SDA)
        else:
            self.i2c = i2c
            
        # Initialize BNO085
        self.bno = BNO08X_I2C(self.i2c)
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        
        print("BNO085 RealIMU initialized successfully")

    def get_heading_rad(self):
        """Get heading in radians [0, 2π) from quaternion.
        
        Returns heading where:
        - 0 radians = North (0°)
        - π/2 radians = West (90° counter-clockwise from north)
        - π radians = South (180° counter-clockwise from north)
        - 3π/2 radians = East (270° counter-clockwise from north)
        
        All angles are measured counter-clockwise from north.
        """
        qi, qj, qk, qr = self.bno.quaternion
        
        # Extract yaw from quaternion
        yaw = math.atan2(2.0 * (qr * qk + qi * qj), 1.0 - 2.0 * (qj * qj + qk * qk))
        
        # Convert from BNO085 coordinate system to our counter-clockwise system:
        # BNO085: 0 = +X (east), π/2 = +Y (north), counter-clockwise positive
        # Our system: 0 = north, counter-clockwise positive
        # So we need to rotate by π/2: yaw_bno + π/2 = yaw_ours
        heading = yaw + math.pi / 2.0
        
        # Normalize to [0, 2π)
        return (heading + 2.0 * math.pi) % (2.0 * math.pi)

    def get_heading_deg(self):
        """Get heading in degrees [0, 360) from quaternion.
        
        Returns heading where:
        - 0° = North
        - 90° = West (counter-clockwise from north)
        - 180° = South (counter-clockwise from north)
        - 270° = East (counter-clockwise from north)
        """
        return math.degrees(self.get_heading_rad())

    def get_accelerometer(self):
        """Get accelerometer readings in m/s²."""
        return self.bno.acceleration

    def get_gyroscope(self):
        """Get gyroscope readings in rad/s."""
        return self.bno.gyro

    def get_magnetometer(self):
        """Get magnetometer readings in μT."""
        return self.bno.magnetic

    def get_quaternion(self):
        """Get quaternion [qi, qj, qk, qr]."""
        return self.bno.quaternion

    def get_rotation_vector(self):
        """Get rotation vector [i, j, k, real]."""
        return self.bno.quaternion


if __name__ == '__main__':
    # Test the RealIMU class
    print("Testing RealIMU class...")
    imu = RealIMU()
    
    try:
        while True:
            heading_rad = imu.get_heading_rad()
            heading_deg = imu.get_heading_deg()
            accel = imu.get_accelerometer()
            gyro = imu.get_gyroscope()
            mag = imu.get_magnetometer()
            
            print(f"Heading: {heading_rad:.3f} rad ({heading_deg:.1f}°)")
            print(f"Accel: x={accel[0]:.2f}, y={accel[1]:.2f}, z={accel[2]:.2f} m/s²")
            print(f"Gyro: x={gyro[0]:.3f}, y={gyro[1]:.3f}, z={gyro[2]:.3f} rad/s")
            print(f"Mag: x={mag[0]:.1f}, y={mag[1]:.1f}, z={mag[2]:.1f} μT")
            print("-" * 50)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nStopped")
