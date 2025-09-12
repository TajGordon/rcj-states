import json
import threading
import time

try:
    import board
    import busio
    HARDWARE_AVAILABLE = True
except Exception:
    HARDWARE_AVAILABLE = False

from flask import Flask, render_template
from flask_sock import Sock

try:
    import config
    from tof_stuff import ToFManager
except Exception:
    config = None
    ToFManager = None


class ToFReader:
    def __init__(self):
        self.readings_lock = threading.Lock()
        self.latest_readings = {}
        self.running = False
        self.thread = None

    def start(self):
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False

    def get_snapshot(self):
        with self.readings_lock:
            return dict(self.latest_readings)

    def _run(self):
        if HARDWARE_AVAILABLE and ToFManager is not None and config is not None:
            i2c = busio.I2C(board.SCL, board.SDA)
            while not i2c.try_lock():
                time.sleep(0.001)
            try:
                mgr = ToFManager(i2c)
                while self.running:
                    data = mgr.batch_current()
                    snapshot = {
                        addr: {
                            "address": addr,
                            "distance_mm": reading.distance_mm,
                            "angle_rad": reading.angle_rad,
                        }
                        for addr, reading in data.items()
                    }
                    with self.readings_lock:
                        self.latest_readings = snapshot
                    time.sleep(0.05)
            finally:
                i2c.unlock()
        else:
            # Mock generator for non-hardware environments
            import math
            angles = [i * (math.pi / 4) for i in range(8)]
            t = 0.0
            while self.running:
                snapshot = {}
                for idx, ang in enumerate(angles):
                    # oscillate distances between 100 and 1500 mm
                    dist = 800 + 600 * (0.5 * (1 + math.sin(t + idx)))
                    snapshot[80 + idx] = {
                        "address": 80 + idx,
                        "distance_mm": int(dist),
                        "angle_rad": ang,
                    }
                with self.readings_lock:
                    self.latest_readings = snapshot
                t += 0.1
                time.sleep(0.05)


app = Flask(__name__)
sock = Sock(app)
tof_reader = ToFReader()
tof_reader.start()


@app.route("/")
def index():
    return render_template("tof_viewer.html")


@sock.route("/ws")
def ws_feed(ws):
    # Push latest snapshot periodically to the client
    try:
        while True:
            snapshot = tof_reader.get_snapshot()
            payload = {
                "type": "tof_readings",
                "data": list(snapshot.values()),
            }
            ws.send(json.dumps(payload))
            time.sleep(0.1)
    except Exception:
        # client disconnected or error; just exit handler
        return


def main():
    # Bind to all interfaces so LAN clients can connect
    app.run(host="0.0.0.0", port=5050, debug=False, threaded=True)


if __name__ == "__main__":
    main()


