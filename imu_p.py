import threading
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
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self.bno = None
        self.i2c = None

        # Initialize BNO085 with timeout and better I2C handling
        try:
            print("Initializing BNO085 IMU...")
            
            # Create new I2C instance if not provided
            if i2c is None:
                self.i2c = busio.I2C(board.SCL, board.SDA)
                # Try to lock with timeout
                start_time = time.time()
                while not self.i2c.try_lock():
                    if time.time() - start_time > 2.0:  # 2 second timeout
                        raise TimeoutError("I2C bus locked by another process")
                    time.sleep(0.01)
                print("I2C bus locked successfully")
            else:
                self.i2c = i2c
            
            # Try to initialize BNO085 with timeout
            import signal
            
            def timeout_handler(signum, frame):
                raise TimeoutError("BNO085 initialization timeout")
            
            signal.signal(signal.SIGALRM, timeout_handler)
            signal.alarm(3)  # 3 second timeout
            
            try:
                self.bno = BNO08X_I2C(self.i2c)
                self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
                self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
                self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
                self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
                signal.alarm(0)  # Cancel timeout
                print("BNO085 IMU initialized successfully")
            except TimeoutError:
                signal.alarm(0)
                raise TimeoutError("BNO085 not responding - check connections")
                
        except Exception as e:
            print(f"BNO085 IMU initialization failed: {e}")
            print("IMU will return mock data")
            self.bno = None
            if self.i2c and i2c is None:  # Only unlock if we created the I2C
                try:
                    self.i2c.unlock()
                except:
                    pass

        # Heading state
        self._heading_deg = 0.0
        self._initial_heading_deg = None

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def _run(self):
        while self._running:
            try:
                h = self.read_heading_deg()
                with self._lock:
                    self._heading_deg = h
            except Exception:
                pass
            time.sleep(0.01)

    def read_heading_deg(self):
        """Return absolute heading in degrees [0,360), derived from rotation vector."""
        if self.bno is None:
            # Return mock heading that slowly rotates
            return (time.time() * 0.1 * 180.0 / math.pi) % 360.0
        
        try:
            qi, qj, qk, qr = self.bno.quaternion
            # yaw from quaternion (right-handed ENU -> heading)
            yaw = math.atan2(2.0 * (qr * qk + qi * qj), 1.0 - 2.0 * (qj * qj + qk * qk))
            heading = math.degrees(yaw)
            if heading < 0:
                heading += 360.0
            return heading
        except Exception as e:
            print(f"Error reading IMU heading: {e}")
            return 0.0

    def read_heading_rad(self):
        """Return absolute heading in radians [0, 2π)."""
        if self.bno is None:
            # Return mock heading that slowly rotates
            return (time.time() * 0.1) % (2.0 * math.pi)
        
        try:
            qi, qj, qk, qr = self.bno.quaternion
            yaw = math.atan2(2.0 * (qr * qk + qi * qj), 1.0 - 2.0 * (qj * qj + qk * qk))
            # normalize to [0, 2π)
            if yaw < 0:
                yaw += 2.0 * math.pi
            return yaw
        except Exception as e:
            print(f"Error reading IMU heading: {e}")
            return 0.0

    def read_relative_heading_deg(self):
        """Heading relative to first measurement (0 = initial orientation)."""
        current = self.read_heading_deg()
        if self._initial_heading_deg is None:
            self._initial_heading_deg = current
        rel = current - self._initial_heading_deg
        if rel < 0:
            rel += 360.0
        elif rel >= 360.0:
            rel -= 360.0
        return rel

    def read_relative_heading_rad(self):
        """Heading in radians relative to first measurement [0, 2π)."""
        current = self.read_heading_rad()
        if self._initial_heading_deg is None:
            # initialize from current radians
            self._initial_heading_deg = math.degrees(current)
        initial_rad = math.radians(self._initial_heading_deg)
        rel = current - initial_rad
        # wrap to [0, 2π)
        rel = (rel + 2.0 * math.pi) % (2.0 * math.pi)
        return rel

    def get_cached_heading_deg(self):
        with self._lock:
            return self._heading_deg

    def get_cached_heading_rad(self):
        with self._lock:
            # convert cached degrees to radians
            return (self._heading_deg or 0.0) * math.pi / 180.0


if __name__ == '__main__':
    # quick test
    i2c = busio.I2C(board.SCL, board.SDA)
    while not i2c.try_lock():
        pass
    try:
        imu = IMU(i2c)
        imu.start()
        for _ in range(200):
            print(f"heading_deg={imu.get_cached_heading_deg():.1f} rel_deg={imu.read_relative_heading_deg():.1f} heading_rad={imu.get_cached_heading_rad():.3f} rel_rad={imu.read_relative_heading_rad():.3f}")
            time.sleep(0.05)
    finally:
        i2c.unlock()