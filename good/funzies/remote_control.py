import cv2
import numpy as np
import time
import math
import threading
import queue
from flask import Flask, render_template, Response, jsonify
from flask_socketio import SocketIO, emit
import json

class RemoteControl:
    def __init__(self, bot, host='0.0.0.0', port=5001):
        """
        Remote control interface for the robot.
        
        Args:
            bot: Bot instance with all components
            host: Host address for web server
            port: Port for web server
        """
        self.bot = bot
        self.host = host
        self.port = port
        
        # Control state
        self.control_state = {
            'forward': False,
            'backward': False,
            'left': False,
            'right': False,
            'turn_left': False,
            'turn_right': False,
            'speed_level': 5  # Default speed level
        }
        
        # Data collection
        self.robot_data = {
            'position': (0, 0),
            'heading': 0.0,
            'ball_detected': False,
            'ball_position': (0, 0),
            'ball_angle': 0.0,
            'ball_distance': 0.0,
            'tof_readings': [],
            'localization_error': 0.0,
            'last_update': time.time()
        }
        
        # Camera frame queue
        self.frame_queue = queue.Queue(maxsize=2)
        
        # Flask app setup
        self.app = Flask(__name__, template_folder='templates')
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        # Threading
        self.running = False
        self.data_thread = None
        self.control_thread = None
        
        # Setup routes and socket events
        self._setup_routes()
        self._setup_socket_events()
        
    def _setup_routes(self):
        """Setup Flask routes."""
        @self.app.route('/')
        def index():
            return render_template('remote_control.html')
        
        @self.app.route('/video_feed')
        def video_feed():
            return Response(self._generate_frames(), 
                          mimetype='multipart/x-mixed-replace; boundary=frame')
        
        @self.app.route('/robot_data')
        def robot_data():
            return jsonify(self.robot_data)
    
    def _setup_socket_events(self):
        """Setup SocketIO events for real-time control."""
        @self.socketio.on('connect')
        def handle_connect():
            print('Remote control client connected')
            emit('status', {'message': 'Connected to robot'})
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            print('Remote control client disconnected')
        
        @self.socketio.on('control_input')
        def handle_control(data):
            """Handle control input from web interface."""
            if 'key' in data and 'action' in data:
                key = data['key']
                action = data['action']  # 'press' or 'release'
                
                if action == 'press':
                    if key == 'w':
                        self.control_state['forward'] = True
                    elif key == 's':
                        self.control_state['backward'] = True
                    elif key == 'a':
                        self.control_state['left'] = True
                    elif key == 'd':
                        self.control_state['right'] = True
                    elif key == 'q':
                        self.control_state['turn_left'] = True
                    elif key == 'e':
                        self.control_state['turn_right'] = True
                elif action == 'release':
                    if key == 'w':
                        self.control_state['forward'] = False
                    elif key == 's':
                        self.control_state['backward'] = False
                    elif key == 'a':
                        self.control_state['left'] = False
                    elif key == 'd':
                        self.control_state['right'] = False
                    elif key == 'q':
                        self.control_state['turn_left'] = False
                    elif key == 'e':
                        self.control_state['turn_right'] = False
        
        @self.socketio.on('speed_change')
        def handle_speed_change(data):
            """Handle speed level change."""
            if 'speed' in data:
                self.control_state['speed_level'] = max(1, min(8, int(data['speed'])))
    
    def _generate_frames(self):
        """Generate video frames for streaming."""
        while self.running:
            try:
                # Get frame from camera
                frame = self.bot.camera.get_processed_frame()
                if frame is not None:
                    # Add overlay information
                    frame = self._add_overlay(frame)
                    
                    # Encode frame
                    ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if ret:
                        frame_bytes = buffer.tobytes()
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                else:
                    # Create a placeholder frame if no camera frame available
                    placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(placeholder, "Camera not available", (200, 240), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    ret, buffer = cv2.imencode('.jpg', placeholder, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if ret:
                        frame_bytes = buffer.tobytes()
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                time.sleep(0.033)  # ~30 FPS
            except Exception as e:
                print(f"Error generating frame: {e}")
                time.sleep(0.1)
    
    def _add_overlay(self, frame):
        """Add information overlay to camera frame."""
        # Get current robot data
        ball_detected = self.bot.camera.is_ball_detected()
        ball_x, ball_y = self.bot.camera.get_ball_position()
        ball_distance = self.bot.camera.get_ball_distance_from_center()
        
        # Add text overlay
        y_offset = 30
        cv2.putText(frame, f"Remote Control Mode", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        y_offset += 30
        cv2.putText(frame, f"Speed Level: {self.control_state['speed_level']}", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        y_offset += 25
        if ball_detected:
            cv2.putText(frame, f"Ball: ({ball_x}, {ball_y}) Dist: {ball_distance:.0f}px", (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "No ball detected", (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # Add control state indicators
        y_offset += 30
        controls = []
        if self.control_state['forward']: controls.append("W")
        if self.control_state['backward']: controls.append("S")
        if self.control_state['left']: controls.append("A")
        if self.control_state['right']: controls.append("D")
        if self.control_state['turn_left']: controls.append("Q")
        if self.control_state['turn_right']: controls.append("E")
        
        if controls:
            cv2.putText(frame, f"Controls: {' '.join(controls)}", (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        return frame
    
    def _collect_robot_data(self):
        """Collect robot data in background thread."""
        while self.running:
            try:
                # Get robot position and heading
                robot_heading = self.bot.imu.get_heading_rad()
                tof_pairs = self.bot.tof.get_localization_pairs(fresh=False)
                robot_x, robot_y, _ = self.bot.localization.estimate_position(robot_heading, tof_pairs)
                
                # Get ball data
                ball_detected = self.bot.camera.is_ball_detected()
                ball_x, ball_y = self.bot.camera.get_ball_position()
                ball_angle = self.bot.camera.get_ball_angle()
                ball_distance = self.bot.camera.get_ball_distance_from_center()
                
                # Get ToF readings
                tof_readings = []
                for angle, distance in tof_pairs:
                    tof_readings.append({
                        'angle': math.degrees(angle),
                        'distance': distance
                    })
                
                # Update robot data
                self.robot_data.update({
                    'position': (robot_x, robot_y),
                    'heading': math.degrees(robot_heading),
                    'ball_detected': ball_detected,
                    'ball_position': (ball_x, ball_y) if ball_detected else (0, 0),
                    'ball_angle': math.degrees(ball_angle) if ball_detected else 0.0,
                    'ball_distance': ball_distance if ball_detected else 0.0,
                    'tof_readings': tof_readings,
                    'localization_error': self.bot.localization.best_error,
                    'last_update': time.time()
                })
                
                # Broadcast data to connected clients
                self.socketio.emit('robot_data', self.robot_data)
                
            except Exception as e:
                print(f"Error collecting robot data: {e}")
            
            time.sleep(0.1)  # 10 Hz data collection
    
    def _control_loop(self):
        """Main control loop for robot movement."""
        while self.running:
            try:
                # Calculate movement based on control state
                forward_speed = 0
                strafe_speed = 0
                turn_speed = 0
                
                # Forward/backward
                if self.control_state['forward']:
                    forward_speed = 1
                elif self.control_state['backward']:
                    forward_speed = -1
                
                # Left/right strafe
                if self.control_state['left']:
                    strafe_speed = 1
                elif self.control_state['right']:
                    strafe_speed = -1
                
                # Turning
                if self.control_state['turn_left']:
                    turn_speed = 1
                elif self.control_state['turn_right']:
                    turn_speed = -1
                
                # Apply movement
                if forward_speed != 0 or strafe_speed != 0:
                    # Calculate direction in degrees
                    direction = math.degrees(math.atan2(strafe_speed, forward_speed))
                    if direction < 0:
                        direction += 360
                    
                    # Calculate speed based on speed level
                    speed_level = self.control_state['speed_level']
                    self.bot.motor_controller.move_direction(direction, speed_level)
                elif turn_speed != 0:
                    # Pure turning
                    direction = 'left' if turn_speed > 0 else 'right'
                    speed_level = self.control_state['speed_level']
                    self.bot.motor_controller.turn_in_place(direction, speed_level)
                else:
                    # No input - stop motors
                    self.bot.motor_controller.stop_motors()
                
            except Exception as e:
                print(f"Error in control loop: {e}")
            
            time.sleep(0.05)  # 20 Hz control loop
    
    def start(self):
        """Start the remote control interface."""
        print(f"Starting Remote Control on http://{self.host}:{self.port}")
        
        self.running = True
        
        # Start background threads
        self.data_thread = threading.Thread(target=self._collect_robot_data)
        self.data_thread.daemon = True
        self.data_thread.start()
        
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        # Start camera if not already running
        if not self.bot.camera.is_running:
            self.bot.camera.start()
        
        # Run Flask app
        try:
            self.socketio.run(self.app, host=self.host, port=self.port, debug=False)
        except KeyboardInterrupt:
            print("Remote control interrupted")
        finally:
            self.stop()
    
    def stop(self):
        """Stop the remote control interface."""
        self.running = False
        self.bot.motor_controller.stop_motors()
        print("Remote control stopped")


def main():
    """Main function to run remote control."""
    import sys
    import os
    
    # Add parent directory to path to import bot components
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    
    from bot import Bot
    
    print("Initializing robot for remote control...")
    bot = Bot()
    
    print("Starting remote control interface...")
    remote_control = RemoteControl(bot)
    remote_control.start()


if __name__ == "__main__":
    main()
