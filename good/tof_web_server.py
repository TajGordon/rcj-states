import json
import threading
import time
import math
import os

from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO, emit

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
        self.error_message = None

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

    def get_error(self):
        with self.lock:
            return self.error_message

    def _run(self):
        try:
            if HARDWARE and ToFArray is not None:
                i2c = busio.I2C(board.SCL, board.SDA)
                while not i2c.try_lock():
                    time.sleep(0.001)
                try:
                    arr = ToFArray(i2c)
                    while self.running:
                        try:
                            pairs = arr.get_localization_pairs(fresh=False)
                            with self.lock:
                                # convert to objects for the frontend
                                self.latest = [
                                    {"angle_rad": a, "distance_mm": d} for (a, d) in pairs
                                ]
                                self.error_message = None
                        except Exception as e:
                            with self.lock:
                                self.error_message = f"TOF reading error: {str(e)}"
                        time.sleep(0.02)
                finally:
                    i2c.unlock()
            else:
                # mock stream for development on non-hardware machines
                t = 0.0
                # Updated angles to match config.py (counter-clockwise from north)
                angles = [0, math.radians(55), math.radians(90), math.radians(125), 
                         math.radians(180), math.radians(215), math.radians(270), math.radians(305)]
                while self.running:
                    items = []
                    for i, a in enumerate(angles):
                        dist = 800 + 600 * (0.5 * (1 + math.sin(t + i * 0.6)))
                        items.append({"angle_rad": a, "distance_mm": int(dist)})
                    with self.lock:
                        self.latest = items
                        self.error_message = None
                    t += 0.1
                    time.sleep(0.02)
        except Exception as e:
            with self.lock:
                self.error_message = f"TOF initialization error: {str(e)}"


class ToFWebServer:
    def __init__(self, host='0.0.0.0', port=5050):
        self.host = host
        self.port = port
        
        # Initialize Flask app with SocketIO
        self.app = Flask(__name__, template_folder='templates')
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        # Initialize TOF stream
        self.stream = ToFStream()
        self.connected_clients = 0
        
        # Setup routes and socket events
        self._setup_routes()
        self._setup_socket_events()
        
        # Start data broadcasting thread
        self.data_thread = threading.Thread(target=self._broadcast_data)
        self.data_thread.daemon = True

    def _setup_routes(self):
        """Setup Flask routes."""
        
        @self.app.route('/')
        def index():
            """Main page with TOF viewer."""
            return render_template('tof_viewer.html')
        
        @self.app.route('/api/status')
        def api_status():
            """API endpoint for TOF status."""
            try:
                error = self.stream.get_error()
                return jsonify({
                    'connected': not bool(error),
                    'error': error,
                    'hardware_available': HARDWARE,
                    'tof_available': ToFArray is not None
                })
            except Exception as e:
                return jsonify({'error': str(e), 'connected': False})
        
        @self.app.route('/api/tof_data')
        def api_tof_data():
            """API endpoint for TOF data."""
            try:
                data = self.stream.snapshot()
                return jsonify({
                    'readings': data,
                    'count': len(data)
                })
            except Exception as e:
                return jsonify({'error': str(e)})

    def _setup_socket_events(self):
        """Setup WebSocket events."""
        
        @self.socketio.on('connect')
        def handle_connect():
            """Handle client connection."""
            self.connected_clients += 1
            print(f"TOF client connected. Total clients: {self.connected_clients}")
            
            # Send initial data
            self._send_tof_data()
            self._send_status()
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            """Handle client disconnection."""
            self.connected_clients -= 1
            print(f"TOF client disconnected. Total clients: {self.connected_clients}")
        
        @self.socketio.on('request_data')
        def handle_data_request():
            """Handle data request from client."""
            self._send_tof_data()
            self._send_status()

    def _send_tof_data(self):
        """Send TOF data via WebSocket."""
        try:
            data = self.stream.snapshot()
            self.socketio.emit('tof_data', {
                'readings': data,
                'count': len(data)
            })
        except Exception as e:
            self.socketio.emit('tof_data', {'error': str(e)})

    def _send_status(self):
        """Send status via WebSocket."""
        try:
            error = self.stream.get_error()
            self.socketio.emit('tof_status', {
                'connected': not bool(error),
                'error': error,
                'hardware_available': HARDWARE,
                'tof_available': ToFArray is not None
            })
        except Exception as e:
            self.socketio.emit('tof_status', {'error': str(e)})

    def _broadcast_data(self):
        """Continuously broadcast data to connected clients."""
        while True:
            try:
                if self.connected_clients > 0:
                    self._send_tof_data()
                    self._send_status()
                time.sleep(0.1)  # 10 Hz update rate
            except Exception as e:
                print(f"Error broadcasting TOF data: {e}")
                time.sleep(1)

    def start(self):
        """Start the web server and TOF stream."""
        try:
            print(f"Starting TOF Web Server on http://{self.host}:{self.port}")
            print("Initializing TOF stream...")
            
            # Start TOF stream
            self.stream.start()
            
            # Start data broadcasting thread
            self.data_thread.start()
            
            print("TOF stream initialized successfully")
            print("Open your web browser and navigate to the URL above to view the TOF data")
            print("Press Ctrl+C to stop the server")
            
            # Run Flask app with SocketIO
            self.socketio.run(self.app, host=self.host, port=self.port, debug=False)
            
        except KeyboardInterrupt:
            print("\nShutting down TOF web server...")
            self.stop()
        except Exception as e:
            print(f"Error starting TOF web server: {e}")
            self.stop()

    def stop(self):
        """Stop the web server and TOF stream."""
        try:
            print("Stopping TOF stream...")
            self.stream.stop()
            print("TOF web server stopped")
        except Exception as e:
            print(f"Error stopping TOF web server: {e}")

def main():
    """Main function to run the TOF web server."""
    try:
        # Create and start web server
        server = ToFWebServer(host='0.0.0.0', port=5050)
        server.start()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()


