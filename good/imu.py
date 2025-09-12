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
        if i2c is None:
            self.i2c = busio.I2C(board.SCL, board.SDA)
        else:
            self.i2c = i2c
        
        self.bno = BNO08X_I2C(self.i2c)
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        
        print("BNO085 IMU initialized successfully")
        
    def get_heading_rad(self):
        qi, qj, qk, qr = self.bno.quaternion
        yaw = math.atan2(2.0 * (qr * qk + qi * qj), 1.0 - 2.0 * (qj * qj + qk * qk))
        heading = yaw + math.pi / 2.0
        return (heading + 2.0 * math.pi) % (2.0 * math.pi)
    

# testing
if __name__ == "__main__":
    print('testing the imu class')
    imu = IMU()
    try:
        while True:
            heading_rad = imu.get_heading_rad()
            heading_deg = math.degrees(heading_rad)
            print(f'heading_rad: {heading_rad:.3f} heading_deg: {heading_deg:.1f}')
            time.sleep(0.1)
    except KeyboardInterrupt:
        print('\nstopped')