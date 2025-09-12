import json
import threading
import time

from flask import Flask, render_template
from flask_sock import Sock

try:
    import board
    import busio
    HARDWARE = True
except Exception:
    HARDWARE = False

try:
    from tof_stuff import ToFArray
except Exception:
    ToFArray = None


class ToFStream:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest = []
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

    def snapshot(self):
        with self.lock:
            return list(self.latest)

    def _run(self):
        if HARDWARE and ToFArray is not None:
            i2c = busio.I2C(board.SCL, board.SDA)
            while not i2c.try_lock():
                time.sleep(0.001)
            try:
                arr = ToFArray(i2c)
                while self.running:
                    pairs = arr.get_localization_pairs(fresh=False)
                    with self.lock:
                        # convert to objects for the frontend
                        self.latest = [
                            {"angle_rad": a, "distance_mm": d} for (a, d) in pairs
                        ]
                    time.sleep(0.05)
            finally:
                i2c.unlock()
        else:
            # mock stream for development on non-hardware machines
            import math
            t = 0.0
            angles = [0, math.radians(35), math.radians(90), math.radians(125), math.radians(180), math.radians(215), math.radians(270), math.radians(305)]
            while self.running:
                items = []
                for i, a in enumerate(angles):
                    dist = 800 + 600 * (0.5 * (1 + math.sin(t + i * 0.6)))
                    items.append({"angle_rad": a, "distance_mm": int(dist)})
                with self.lock:
                    self.latest = items
                t += 0.1
                time.sleep(0.05)


app = Flask(__name__)
sock = Sock(app)
stream = ToFStream()
stream.start()


@app.route("/")
def index():
    return render_template("tof_viewer.html")


@sock.route("/ws")
def ws(ws):
    try:
        while True:
            payload = {"type": "tof_readings", "data": stream.snapshot()}
            ws.send(json.dumps(payload))
            time.sleep(0.1)
    except Exception:
        return


def main():
    app.run(host="0.0.0.0", port=5050, debug=False, threaded=True)


if __name__ == "__main__":
    main()


