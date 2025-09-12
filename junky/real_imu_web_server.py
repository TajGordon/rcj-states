import json
import threading
import time
import math

from flask import Flask, render_template
from flask_sock import Sock

try:
    import board
    import busio
    HARDWARE = True
except Exception:
    HARDWARE = False

try:
    from real_imu_class import RealIMU
except Exception:
    RealIMU = None


class IMUStream:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest = {}
        self.running = False
        self.thread = None
        self.imu = None

    def start(self):
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False

    def snapshot(self):
        with self.lock:
            return dict(self.latest)

    def _run(self):
        if HARDWARE and RealIMU is not None:
            try:
                self.imu = RealIMU()
                while self.running:
                    # Get all IMU data
                    heading_rad = self.imu.get_heading_rad()
                    heading_deg = self.imu.get_heading_deg()
                    accel = self.imu.get_accelerometer()
                    gyro = self.imu.get_gyroscope()
                    mag = self.imu.get_magnetometer()
                    quat = self.imu.get_quaternion()
                    
                    with self.lock:
                        self.latest = {
                            "heading_rad": heading_rad,
                            "heading_deg": heading_deg,
                            "accel": {
                                "x": accel[0],
                                "y": accel[1], 
                                "z": accel[2]
                            },
                            "gyro": {
                                "x": gyro[0],
                                "y": gyro[1],
                                "z": gyro[2]
                            },
                            "mag": {
                                "x": mag[0],
                                "y": mag[1],
                                "z": mag[2]
                            },
                            "quaternion": {
                                "qi": quat[0],
                                "qj": quat[1],
                                "qk": quat[2],
                                "qr": quat[3]
                            },
                            "timestamp": time.time()
                        }
                    time.sleep(0.02)  # 50Hz update rate
            except Exception as e:
                print(f"Error in IMU stream: {e}")
                with self.lock:
                    self.latest = {"error": str(e)}
        else:
            # Mock stream for development on non-hardware machines
            t = 0.0
            while self.running:
                # Simulate rotating heading
                mock_heading_rad = (t * 0.1) % (2 * math.pi)
                mock_heading_deg = math.degrees(mock_heading_rad)
                
                # Simulate some sensor noise
                accel_x = 0.1 * math.sin(t * 0.5)
                accel_y = 0.1 * math.cos(t * 0.3)
                accel_z = 9.8 + 0.05 * math.sin(t * 0.2)
                
                gyro_x = 0.1 * math.sin(t * 0.4)
                gyro_y = 0.1 * math.cos(t * 0.6)
                gyro_z = 0.1 * math.sin(t * 0.1)
                
                mag_x = 20 + 2 * math.sin(t * 0.1)
                mag_y = 5 + 1 * math.cos(t * 0.15)
                mag_z = 40 + 3 * math.sin(t * 0.08)
                
                with self.lock:
                    self.latest = {
                        "heading_rad": mock_heading_rad,
                        "heading_deg": mock_heading_deg,
                        "accel": {
                            "x": accel_x,
                            "y": accel_y,
                            "z": accel_z
                        },
                        "gyro": {
                            "x": gyro_x,
                            "y": gyro_y,
                            "z": gyro_z
                        },
                        "mag": {
                            "x": mag_x,
                            "y": mag_y,
                            "z": mag_z
                        },
                        "quaternion": {
                            "qi": 0.0,
                            "qj": 0.0,
                            "qk": math.sin(mock_heading_rad / 2),
                            "qr": math.cos(mock_heading_rad / 2)
                        },
                        "timestamp": time.time(),
                        "mock": True
                    }
                t += 0.1
                time.sleep(0.02)


app = Flask(__name__)
sock = Sock(app)
stream = IMUStream()
stream.start()


@app.route("/")
def index():
    return render_template("real_imu_viewer.html")


@sock.route("/ws")
def ws(ws):
    try:
        while True:
            payload = {"type": "imu_data", "data": stream.snapshot()}
            ws.send(json.dumps(payload))
            time.sleep(0.05)  # 20Hz update rate for websocket
    except Exception as e:
        print(f"WebSocket error: {e}")
        return


def main():
    print("Starting Real IMU Web Server (Counter-Clockwise)...")
    print("Open http://localhost:5051 in your browser")
    app.run(host="0.0.0.0", port=5051, debug=False, threaded=True)


if __name__ == "__main__":
    main()
