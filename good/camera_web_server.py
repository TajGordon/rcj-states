import cv2
import time
import threading
import math
import json
from flask import Flask, render_template, Response, jsonify
from flask_socketio import SocketIO, emit
from camera import Camera

class CameraWebServer:
    """
    Web server that uses the Camera module to stream camera feed with OpenCV annotations.
    This server doesn't modify the camera.py file but uses its interface.
    """
    
    def __init__(self, host='0.0.0.0', port=5000):
        """
        Initialize the web server.
        
        Args:
            host: Host address to bind to
            port: Port to listen on
        """
        self.host = host
        self.port = port
        
        # Initialize camera with frame queue enabled for web streaming and enhanced detection
        self.camera = Camera(enable_frame_queue=True, detection_mode='enhanced')
        
        # Flask app with SocketIO
        self.app = Flask(__name__, template_folder='templates')
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        # Frame processing
        self.current_frame = None
        self.frame_lock = threading.Lock()
        
        # Statistics
        self.frame_count = 0
        self.start_time = time.time()
        self.last_fps_update = time.time()
        self.current_fps = 0
        
        # WebSocket clients
        self.connected_clients = 0
        
        # Setup routes and socket events
        self._setup_routes()
        self._setup_socket_events()
        
        # Start frame processing thread
        self.frame_thread = threading.Thread(target=self._process_frames)
        self.frame_thread.daemon = True
        
        # Start data broadcasting thread
        self.data_thread = threading.Thread(target=self._broadcast_data)
        self.data_thread.daemon = True
        
    def _setup_routes(self):
        """Setup Flask routes for the web interface."""
        
        @self.app.route('/')
        def index():
            """Main page with camera viewer."""
            return render_template('camera_viewer.html')
        
        @self.app.route('/video_feed')
        def video_feed():
            """Video streaming endpoint."""
            return Response(self._generate_frames(),
                          mimetype='multipart/x-mixed-replace; boundary=frame')
        
        @self.app.route('/api/start_camera')
        def api_start_camera():
            """API endpoint to start the camera."""
            try:
                if not self.camera.is_running:
                    self.camera.start()
                    return jsonify({'success': True, 'message': 'Camera started'})
                else:
                    return jsonify({'success': False, 'message': 'Camera already running'})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/stop_camera')
        def api_stop_camera():
            """API endpoint to stop the camera."""
            try:
                if self.camera.is_running:
                    self.camera.stop()
                    return jsonify({'success': True, 'message': 'Camera stopped'})
                else:
                    return jsonify({'success': False, 'message': 'Camera not running'})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})
        
    
    def _setup_socket_events(self):
        """Setup WebSocket events."""
        
        @self.socketio.on('connect')
        def handle_connect():
            """Handle client connection."""
            self.connected_clients += 1
            print(f"Client connected. Total clients: {self.connected_clients}")
            
            # Send initial data
            self._send_camera_status()
            self._send_ball_data()
            self._send_camera_info()
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            """Handle client disconnection."""
            self.connected_clients -= 1
            print(f"Client disconnected. Total clients: {self.connected_clients}")
        
        @self.socketio.on('request_data')
        def handle_data_request():
            """Handle data request from client."""
            self._send_camera_status()
            self._send_ball_data()
            self._send_camera_info()
    
    def _send_camera_status(self):
        """Send camera status via WebSocket."""
        try:
            camera_info = self.camera.get_camera_info()
            status_data = {
                'connected': camera_info['is_running'],
                'has_frame': self.current_frame is not None,
                'camera_status': camera_info
            }
            self.socketio.emit('camera_status', status_data)
        except Exception as e:
            self.socketio.emit('camera_status', {'error': str(e), 'connected': False, 'has_frame': False})
    
    def _send_ball_data(self):
        """Send ball detection data via WebSocket."""
        try:
            ball_x, ball_y = self.camera.get_ball_position()
            ball_radius = self.camera.get_ball_radius()
            ball_angle = self.camera.get_ball_angle()
            ball_detected = self.camera.is_ball_detected()
            
            if ball_detected and ball_x is not None and ball_y is not None:
                # Calculate area from radius
                area = 3.14159 * (ball_radius ** 2)
                # Convert angle to degrees for display
                angle_degrees = math.degrees(ball_angle)
                ball_data = {
                    'detected': True,
                    'center': [ball_x, ball_y],
                    'radius': ball_radius,
                    'area': area,
                    'angle_rad': ball_angle,
                    'angle_deg': angle_degrees
                }
            else:
                ball_data = {
                    'detected': False,
                    'center': [None, None],
                    'radius': 0,
                    'area': 0,
                    'angle_rad': 0.0,
                    'angle_deg': 0.0
                }
            
            self.socketio.emit('ball_data', ball_data)
        except Exception as e:
            self.socketio.emit('ball_data', {'error': str(e)})
    
    def _send_camera_info(self):
        """Send camera information via WebSocket."""
        try:
            camera_info = self.camera.get_camera_info()
            info_data = {
                'width': 640,  # Camera resolution from camera.py
                'height': 640,  # Updated to match your change
                'fps': self.current_fps,
                'is_running': camera_info['is_running'],
                'frame_queue_available': camera_info['frame_queue_available'],
                'detection_mode': camera_info.get('detection_mode', 'unknown')
            }
            self.socketio.emit('camera_info', info_data)
        except Exception as e:
            self.socketio.emit('camera_info', {'error': str(e)})
    
    def _broadcast_data(self):
        """Continuously broadcast data to connected clients."""
        while True:
            try:
                if self.connected_clients > 0:
                    self._send_camera_status()
                    self._send_ball_data()
                    self._send_camera_info()
                time.sleep(0.1)  # 10 Hz update rate
            except Exception as e:
                print(f"Error broadcasting data: {e}")
                time.sleep(1)
    
    def _process_frames(self):
        """Process frames from the camera module and add additional annotations."""
        while True:
            try:
                # Get processed frame from camera module
                frame = self.camera.get_processed_frame()
                
                if frame is not None:
                    # Add additional annotations
                    annotated_frame = self._add_annotations(frame)
                    
                    with self.frame_lock:
                        self.current_frame = annotated_frame
                    
                    # Update FPS calculation
                    self._update_fps()
                else:
                    time.sleep(0.01)  # Small delay if no frame available
                    
            except Exception as e:
                print(f"Error processing frame: {e}")
                time.sleep(0.1)
    
    def _add_annotations(self, frame):
        """
        Add additional annotations to the frame.
        
        Args:
            frame: Input frame from camera module
            
        Returns:
            Annotated frame
        """
        # Create a copy to avoid modifying the original
        annotated_frame = frame.copy()
        
        # Get ball information
        ball_x, ball_y = self.camera.get_ball_position()
        ball_radius = self.camera.get_ball_radius()
        ball_detected = self.camera.is_ball_detected()
        
        # Add FPS counter
        fps_text = f"FPS: {self.current_fps:.1f}"
        cv2.putText(annotated_frame, fps_text, (10, annotated_frame.shape[0] - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Add frame counter
        frame_text = f"Frame: {self.frame_count}"
        cv2.putText(annotated_frame, frame_text, (10, annotated_frame.shape[0] - 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Add timestamp
        timestamp = time.strftime("%H:%M:%S")
        cv2.putText(annotated_frame, timestamp, (annotated_frame.shape[1] - 100, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Add grid lines for better visualization
        height, width = annotated_frame.shape[:2]
        
        # Vertical lines
        for i in range(1, 4):
            x = width * i // 4
            cv2.line(annotated_frame, (x, 0), (x, height), (100, 100, 100), 1)
        
        # Horizontal lines
        for i in range(1, 4):
            y = height * i // 4
            cv2.line(annotated_frame, (0, y), (width, y), (100, 100, 100), 1)
        
        # Add ball tracking information if ball is detected
        if ball_detected and ball_x is not None and ball_y is not None:
            # Add distance from center
            center_x, center_y = width // 2, height // 2
            distance = ((ball_x - center_x) ** 2 + (ball_y - center_y) ** 2) ** 0.5
            
            # Draw line from center to ball
            cv2.line(annotated_frame, (center_x, center_y), (ball_x, ball_y), (0, 255, 255), 2)
            
            # Add distance text
            distance_text = f"Dist: {distance:.1f}px"
            cv2.putText(annotated_frame, distance_text, (ball_x + 20, ball_y - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            # Add angle information (both radians and degrees)
            angle_rad = self.camera.get_ball_angle()
            angle_deg = math.degrees(angle_rad)
            angle_text = f"Angle: {angle_rad:.3f} rad ({angle_deg:.1f}°)"
            cv2.putText(annotated_frame, angle_text, (ball_x + 20, ball_y + 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            # Add angle range indicator (0-2π)
            range_text = f"Range: 0-{2*math.pi:.3f} rad (0-360°)"
            cv2.putText(annotated_frame, range_text, (ball_x + 20, ball_y + 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        
        return annotated_frame
    
    def _update_fps(self):
        """Update FPS calculation."""
        self.frame_count += 1
        current_time = time.time()
        
        # Update FPS every second
        if current_time - self.last_fps_update >= 1.0:
            elapsed = current_time - self.last_fps_update
            self.current_fps = (self.frame_count - (self.frame_count - int(elapsed * self.current_fps))) / elapsed
            self.last_fps_update = current_time
    
    def _generate_frames(self):
        """Generate frames for streaming."""
        while True:
            with self.frame_lock:
                if self.current_frame is not None:
                    # Encode frame as JPEG
                    ret, buffer = cv2.imencode('.jpg', self.current_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if ret:
                        frame_bytes = buffer.tobytes()
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            time.sleep(0.033)  # ~30 FPS
    
    def start(self):
        """Start the web server and camera."""
        try:
            print(f"Starting Camera Web Server on http://{self.host}:{self.port}")
            print("Initializing camera...")
            
            # Start camera
            self.camera.start()
            
            # Start frame processing thread
            self.frame_thread.start()
            
            # Start data broadcasting thread
            self.data_thread.start()
            
            print("Camera initialized successfully")
            print("Open your web browser and navigate to the URL above to view the camera feed")
            print("Press Ctrl+C to stop the server")
            
            # Run Flask app with SocketIO
            self.socketio.run(self.app, host=self.host, port=self.port, debug=False)
            
        except KeyboardInterrupt:
            print("\nShutting down web server...")
            self.stop()
        except Exception as e:
            print(f"Error starting web server: {e}")
            self.stop()
    
    def stop(self):
        """Stop the web server and camera."""
        try:
            print("Stopping camera...")
            self.camera.stop()
            print("Web server stopped")
        except Exception as e:
            print(f"Error stopping web server: {e}")

def main():
    """Main function to run the web server."""
    try:
        # Create and start web server
        server = CameraWebServer(host='0.0.0.0', port=5000)
        server.start()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
