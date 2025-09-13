#!/usr/bin/env python3
"""
Flask web server for soccer_bot.py to view camera feed and ball detection in real-time.
"""

from flask import Flask, render_template, Response, jsonify
import cv2
import numpy as np
import time
import threading
from picamera2 import Picamera2
import queue

app = Flask(__name__)

class SoccerBotWebServer:
    def __init__(self):
        # Initialize camera
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_video_configuration(
            main={"size": (640, 480), "format": "RGB888"}
        ))
        self.picam2.start()
        
        # HSV range for orange ball detection (same as soccer_bot.py)
        self.lower_orange = np.array([0, 50, 30])
        self.upper_orange = np.array([25, 255, 255])
        
        # Frame center
        self.frame_center_x = 320
        self.frame_center_y = 240
        
        # Detection state
        self.ball_detected = False
        self.ball_center = None
        self.ball_radius = 0
        self.orange_pixels = 0
        self.total_contours = 0
        self.filtered_contours = 0
        self.last_detection_time = 0
        
        # Frame queue for video streaming
        self.frame_queue = queue.Queue(maxsize=2)
        
        # Start camera thread
        self.camera_running = True
        self.camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self.camera_thread.start()
        
        print("Soccer Bot Web Server initialized")
    
    def _camera_loop(self):
        """Camera processing loop running in background thread."""
        while self.camera_running:
            try:
                # Capture frame
                frame = self.picam2.capture_array()
                
                # Detect ball
                ball_detected, ball_center, ball_radius, mask, orange_pixels, total_contours, filtered_contours = self._detect_ball(frame)
                
                # Update detection state
                self.ball_detected = ball_detected
                self.ball_center = ball_center
                self.ball_radius = ball_radius
                self.orange_pixels = orange_pixels
                self.total_contours = total_contours
                self.filtered_contours = filtered_contours
                self.last_detection_time = time.time()
                
                # Create overlay frame
                overlay_frame = self._create_overlay_frame(frame, ball_detected, ball_center, ball_radius)
                
                # Try to put frame in queue (non-blocking)
                try:
                    self.frame_queue.put_nowait(overlay_frame)
                except queue.Full:
                    pass  # Skip this frame if queue is full
                
                time.sleep(0.033)  # ~30 FPS
                
            except Exception as e:
                print(f"Error in camera loop: {e}")
                time.sleep(0.1)
    
    def _detect_ball(self, frame):
        """Detect ball in frame and return detection info."""
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Count orange pixels
        orange_pixels = np.sum(mask > 0)
        
        # Filter contours by area
        filtered_contours = [x for x in contours if cv2.contourArea(x) > 5 and cv2.contourArea(x) < 50000]
        
        ball_detected = False
        ball_center = None
        ball_radius = 0
        
        if filtered_contours:
            largest_contour = max(filtered_contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            ball_center = (int(x), int(y))
            ball_radius = int(radius)
            ball_detected = True
        
        return ball_detected, ball_center, ball_radius, mask, orange_pixels, len(contours), len(filtered_contours)
    
    def _create_overlay_frame(self, frame, ball_detected, ball_center, ball_radius):
        """Create frame with ball detection overlay."""
        overlay_frame = frame.copy()
        
        if ball_detected:
            # Draw ball detection
            cv2.circle(overlay_frame, ball_center, ball_radius, (0, 255, 0), 2)
            cv2.circle(overlay_frame, ball_center, 2, (0, 255, 0), -1)
            
            # Draw center crosshair
            cv2.line(overlay_frame, (self.frame_center_x - 10, self.frame_center_y), 
                    (self.frame_center_x + 10, self.frame_center_y), (255, 0, 0), 1)
            cv2.line(overlay_frame, (self.frame_center_x, self.frame_center_y - 10), 
                    (self.frame_center_x, self.frame_center_y + 10), (255, 0, 0), 1)
            
            # Draw line from center to ball
            cv2.line(overlay_frame, (self.frame_center_x, self.frame_center_y), 
                    ball_center, (255, 255, 0), 2)
            
            # Add text info
            cv2.putText(overlay_frame, f"Ball: ({ball_center[0]}, {ball_center[1]})", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(overlay_frame, f"Radius: {ball_radius}", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            # No ball detected
            cv2.putText(overlay_frame, "No ball detected", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        return overlay_frame
    
    def get_latest_frame(self):
        """Get the latest frame from the queue."""
        try:
            return self.frame_queue.get_nowait()
        except queue.Empty:
            return None
    
    def get_detection_data(self):
        """Get current detection data."""
        return {
            'ball_detected': self.ball_detected,
            'ball_center': self.ball_center,
            'ball_radius': self.ball_radius,
            'orange_pixels': self.orange_pixels,
            'total_contours': self.total_contours,
            'filtered_contours': self.filtered_contours,
            'last_detection_time': self.last_detection_time,
            'frame_center': (self.frame_center_x, self.frame_center_y)
        }
    
    def stop(self):
        """Stop the camera and cleanup."""
        self.camera_running = False
        if hasattr(self, 'camera_thread'):
            self.camera_thread.join(timeout=1.0)
        self.picam2.stop()

# Global web server instance
web_server = None

@app.route('/')
def index():
    """Main page with camera feed and detection info."""
    return render_template('soccer_bot_viewer.html')

@app.route('/video_feed')
def video_feed():
    """Video streaming route."""
    def generate():
        while True:
            frame = web_server.get_latest_frame()
            if frame is not None:
                # Encode frame as JPEG
                ret, buffer = cv2.imencode('.jpg', frame)
                if ret:
                    frame_bytes = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            time.sleep(0.033)  # ~30 FPS
    
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/detection_data')
def detection_data():
    """Get current detection data as JSON."""
    return jsonify(web_server.get_detection_data())

@app.route('/control/<action>')
def control(action):
    """Control actions (for future use)."""
    if action == 'stop':
        return jsonify({'status': 'stopped'})
    elif action == 'start':
        return jsonify({'status': 'started'})
    else:
        return jsonify({'status': 'unknown_action'})

def run_server(host='0.0.0.0', port=5000, debug=False):
    """Run the Flask web server."""
    global web_server
    
    print("Starting Soccer Bot Web Server...")
    print(f"Web interface will be available at: http://{host}:{port}")
    print("Press Ctrl+C to stop")
    print("=" * 50)
    
    # Initialize web server
    web_server = SoccerBotWebServer()
    
    try:
        app.run(host=host, port=port, debug=debug, threaded=True)
    except KeyboardInterrupt:
        print("\nShutting down web server...")
    finally:
        if web_server:
            web_server.stop()
        print("Web server stopped.")

if __name__ == "__main__":
    run_server()
