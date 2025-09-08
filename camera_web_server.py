import cv2
import numpy as np
from flask import Flask, render_template, Response
from picamera2 import Picamera2
import threading
import time

class CameraWebServer:
    def __init__(self):
        # Initialize Pi Camera
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_video_configuration(
            main={"size": (640, 480), "format": "RGB888"}
        ))
        self.picam2.start()
        
        # Ball detection parameters
        self.lower_orange = np.array([0, 132, 61])
        self.upper_orange = np.array([14, 255, 255])
        
        # Frame processing
        self.frame = None
        self.frame_lock = threading.Lock()
        
        # Start frame capture thread
        self.capture_thread = threading.Thread(target=self._capture_frames)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        
        # Flask app
        self.app = Flask(__name__)
        self._setup_routes()
    
    def _capture_frames(self):
        """Continuously capture frames from the camera"""
        while True:
            try:
                # Capture frame from Pi Camera
                frame = self.picam2.capture_array()
                
                # Convert to HSV for processing
                hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                
                # Process frame for ball detection
                processed_frame = self._process_frame(hsv_frame, frame)
                
                with self.frame_lock:
                    self.frame = processed_frame
                    
            except Exception as e:
                print(f"Error capturing frame: {e}")
                time.sleep(0.1)
    
    def _process_frame(self, hsv_frame, rgb_frame):
        """Process frame for ball detection and add visual indicators"""
        # Create a copy of RGB frame for display
        display_frame = rgb_frame.copy()
        
        # Ball detection using HSV frame
        mask = cv2.inRange(hsv_frame, self.lower_orange, self.upper_orange)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours by area
        filtered_contours = [x for x in contours if cv2.contourArea(x) > 100 and cv2.contourArea(x) < 30000]
        
        if filtered_contours:
            largest_contour = max(filtered_contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            
            center = (int(x), int(y))
            radius = int(radius)
            
            # Draw circle around detected ball
            cv2.circle(display_frame, center, radius, (0, 255, 0), 2)
            cv2.circle(display_frame, center, 2, (0, 255, 0), -1)
            
            # Add text label
            cv2.putText(display_frame, f"Ball: ({center[0]}, {center[1]})", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(display_frame, "No ball detected", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        # Add center crosshair
        height, width = display_frame.shape[:2]
        center_x, center_y = width // 2, height // 2
        cv2.line(display_frame, (center_x - 20, center_y), (center_x + 20, center_y), (255, 255, 255), 1)
        cv2.line(display_frame, (center_x, center_y - 20), (center_x, center_y + 20), (255, 255, 255), 1)
        
        return display_frame
    
    def _generate_frames(self):
        """Generate frames for streaming"""
        while True:
            with self.frame_lock:
                if self.frame is not None:
                    # Encode frame as JPEG
                    ret, buffer = cv2.imencode('.jpg', self.frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if ret:
                        frame_bytes = buffer.tobytes()
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            time.sleep(0.033)  # ~30 FPS
    
    def _setup_routes(self):
        """Setup Flask routes"""
        
        @self.app.route('/')
        def index():
            return render_template('camera_viewer.html')
        
        @self.app.route('/video_feed')
        def video_feed():
            return Response(self._generate_frames(),
                          mimetype='multipart/x-mixed-replace; boundary=frame')
    
    def run(self, host='0.0.0.0', port=5000, debug=False):
        """Run the web server"""
        print(f"Starting camera web server on http://{host}:{port}")
        print("Open your web browser and navigate to the URL above to view the camera feed")
        self.app.run(host=host, port=port, debug=debug, threaded=True)

def main():
    try:
        server = CameraWebServer()
        server.run(host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        print("\nShutting down web server...")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()

