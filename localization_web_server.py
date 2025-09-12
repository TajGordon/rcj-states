import json
import threading
import time
import math

from flask import Flask, render_template, jsonify
from flask_sock import Sock

try:
    import board
    import busio
    HARDWARE = True
except Exception:
    HARDWARE = False

from localization import FieldMap, load_rust_field_geometry
from IMU import IMU
from tof_stuff import ToFArray


class LocalizationStream:
    def __init__(self):
        self.lock = threading.Lock()
        self.snapshot_data = {}
        self.running = False
        self.thread = None

        # static field geometry (same for all clients)
        field = FieldMap()
        load_rust_field_geometry(field)
        self.field_segments = [
            {"x1": s.x1, "y1": s.y1, "x2": s.x2, "y2": s.y2} for s in field.segments
        ]

        self.field = field

    def start(self):
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False

    def get_snapshot(self):
        with self.lock:
            return dict(self.snapshot_data)

    def _run(self):
        if HARDWARE:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                imu = IMU(i2c)
                tof = ToFArray(i2c)

                # Use field center as a neutral visualization position; client can ignore x,y
                x_mm, y_mm = 1215.0, 910.0

                while self.running:
                    pairs = tof.get_localization_pairs(fresh=False)
                    theta = imu.read_heading_rad()
                    
                    print(f"Debug: pairs={len(pairs)}, theta={theta:.3f}")

                    rays = [
                        {"angle_rad": a, "distance_mm": d}
                        for (a, d) in pairs
                    ]

                    snap = {
                        "field": self.field_segments,
                        "pose": {"x_mm": x_mm, "y_mm": y_mm, "theta_rad": theta},
                        "rays": rays,
                    }
                    with self.lock:
                        self.snapshot_data = snap

                    time.sleep(0.05)
            except Exception as e:
                print(f"Localization hardware stream failed: {e}")
                # fallback to mock loop if hardware init fails
                self._run_mock()
        else:
            # Mock stream for development
            self._run_mock()

    def _run_mock(self):
        print("Running mock localization stream")
        t = 0.0
        pose = {"x_mm": 1215.0, "y_mm": 910.0, "theta_rad": 0.0}
        angles = [0, math.radians(35), math.radians(90), math.radians(125), math.radians(180), math.radians(215), math.radians(270), math.radians(305)]
        while self.running:
            pose["theta_rad"] = (pose["theta_rad"] + 0.01) % (2 * math.pi)
            rays = []
            for i, a in enumerate(angles):
                dist = 1000 + 400 * (0.5 * (1 + math.sin(t + i * 0.7)))
                rays.append({"angle_rad": a, "distance_mm": int(dist)})
            snap = {
                "field": self.field_segments,
                "pose": pose.copy(),
                "rays": rays,
            }
            with self.lock:
                self.snapshot_data = snap
            t += 0.05
            time.sleep(0.05)


app = Flask(__name__)
sock = Sock(app)
stream = LocalizationStream()
stream.start()


@app.route("/")
def index():
    return render_template("localization_viewer.html")

@app.route("/snapshot")
def snapshot_http():
    # Debug endpoint to verify data path without websockets
    return jsonify({"type": "localization", "data": stream.get_snapshot()})

@sock.route("/ws")
def ws(ws):
    try:
        print("WS: client connected")
        while True:
            payload = {"type": "localization", "data": stream.get_snapshot()}
            ws.send(json.dumps(payload))
            time.sleep(0.05)
    except Exception:
        print("WS: client disconnected or error")
        return


def main():
    app.run(host="0.0.0.0", port=6060, debug=False, threaded=True)


if __name__ == "__main__":
    main()


