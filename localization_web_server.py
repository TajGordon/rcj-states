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

from localization import FieldMap, load_rust_field_geometry, LocalizationManager, Pose


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
            i2c = busio.I2C(board.SCL, board.SDA)
            while not i2c.try_lock():
                time.sleep(0.001)
            try:
                lm = LocalizationManager(i2c, self.field)
                # Initial guess: center-ish of field, 0 heading
                current_pose = Pose(1215.0, 910.0, 0.0)

                while self.running:
                    # Get live sensor pairs
                    pairs = lm.get_sensor_pairs(fresh=False)
                    # Heading from IMU
                    theta = lm.imu.read_heading_rad()
                    current_pose = Pose(current_pose.x_mm, current_pose.y_mm, theta)

                    # Optionally refine pose (lock angle to IMU)
                    est = lm.estimate_pose(initial=current_pose, angle_span_rad=0.0)
                    current_pose = est

                    rays = [
                        {"angle_rad": a, "distance_mm": d}
                        for (a, d) in pairs
                    ]

                    snap = {
                        "field": self.field_segments,
                        "pose": {"x_mm": current_pose.x_mm, "y_mm": current_pose.y_mm, "theta_rad": current_pose.theta_rad},
                        "rays": rays,
                    }
                    with self.lock:
                        self.snapshot_data = snap

                    time.sleep(0.05)
            finally:
                i2c.unlock()
        else:
            # Mock stream for development
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


@sock.route("/ws")
def ws(ws):
    try:
        while True:
            ws.send(json.dumps({"type": "localization", "data": stream.get_snapshot()}))
            time.sleep(0.05)
    except Exception:
        return


def main():
    app.run(host="0.0.0.0", port=6060, debug=False, threaded=True)


if __name__ == "__main__":
    main()


