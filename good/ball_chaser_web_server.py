import cv2
import numpy as np
import math
import threading
import time
from flask import Flask, render_template, Response, jsonify
from camera import Camera

class BallChaserWebServer:
    def __init__(self, camera, motor_controller, agent):
        self.camera = camera
        self.motor_controller = motor_controller
        self.agent = agent
        self.app = Flask(__name__)
        self.setup_routes()
        
        # Ball tracking data
        self.ball_data = {
            'detected': False,
            'angle_rad': 0.0,
            'angle_deg': 0.0,
            'distance': 0.0,
            'position': (0, 0),
            'radius': 0
        }
        
        # Movement data
        self.movement_data = {
            'current_speed': 0.0,
            'current_direction': 0.0,
            'movement_state': 'stopped',
            'last_command_time': 0.0
        }
        
    def setup_routes(self):
        """Setup Flask routes"""
        
        @self.app.route('/')
        def index():
            return render_template('ball_chaser_viewer.html')
        
        @self.app.route('/video_feed')
        def video_feed():
            return Response(self.generate_frames(), 
                          mimetype='multipart/x-mixed-replace; boundary=frame')
        
        @self.app.route('/ball_data')
        def get_ball_data():
            return jsonify(self.ball_data)
        
        @self.app.route('/movement_data')
        def get_movement_data():
            return jsonify(self.movement_data)
        
        @self.app.route('/start_chasing')
        def start_chasing():
            if hasattr(self.agent, 'running'):
                self.agent.running = True
                return jsonify({'status': 'started'})
            return jsonify({'status': 'error', 'message': 'Agent not available'})
        
        @self.app.route('/stop_chasing')
        def stop_chasing():
            if hasattr(self.agent, 'running'):
                self.agent.running = False
                self.motor_controller.stop_motors()
                return jsonify({'status': 'stopped'})
            return jsonify({'status': 'error', 'message': 'Agent not available'})
    
    def generate_frames(self):
        """Generate video frames with ball detection overlay"""
        while True:
            try:
                # Get processed frame from camera
                frame = self.camera.get_processed_frame()
                
                if frame is not None:
                    # Get current ball data
                    ball_detected = self.camera.is_ball_detected()
                    
                    if ball_detected:
                        ball_angle = self.camera.get_ball_angle()
                        ball_distance = self.camera.get_ball_distance_from_center()
                        ball_x, ball_y = self.camera.get_ball_position()
                        ball_radius = self.camera.get_ball_radius()
                        
                        # Update ball data
                        self.ball_data.update({
                            'detected': True,
                            'angle_rad': ball_angle,
                            'angle_deg': math.degrees(ball_angle),
                            'distance': ball_distance,
                            'position': (ball_x, ball_y),
                            'radius': ball_radius
                        })
                        
                        # Draw angle line from center to ball
                        center_x, center_y = 320, 320  # Camera center
                        if ball_x is not None and ball_y is not None:
                            # Draw line from center to ball
                            cv2.line(frame, (center_x, center_y), (int(ball_x), int(ball_y)), (0, 255, 0), 3)
                            
                            # Draw angle text
                            angle_text = f"Angle: {math.degrees(ball_angle):.1f}°"
                            cv2.putText(frame, angle_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            
                            # Draw distance text
                            distance_text = f"Distance: {ball_distance:.0f}px"
                            cv2.putText(frame, distance_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            
                            # Draw movement direction arrow
                            self._draw_movement_arrow(frame, ball_angle, center_x, center_y)
                    else:
                        # No ball detected
                        self.ball_data.update({
                            'detected': False,
                            'angle_rad': 0.0,
                            'angle_deg': 0.0,
                            'distance': 0.0,
                            'position': (0, 0),
                            'radius': 0
                        })
                        
                        cv2.putText(frame, "No ball detected", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    
                    # Draw movement status
                    self._draw_movement_status(frame)
                    
                    # Encode frame as JPEG
                    ret, buffer = cv2.imencode('.jpg', frame)
                    if ret:
                        frame_bytes = buffer.tobytes()
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                
                time.sleep(0.033)  # ~30 FPS
                
            except Exception as e:
                print(f"Error generating frame: {e}")
                time.sleep(0.1)
    
    def _draw_movement_arrow(self, frame, ball_angle, center_x, center_y):
        """Draw arrow showing movement direction"""
        # Calculate arrow endpoint (100 pixels from center in ball direction)
        arrow_length = 100
        end_x = int(center_x + arrow_length * math.sin(ball_angle))
        end_y = int(center_y - arrow_length * math.cos(ball_angle))  # Negative because y increases downward
        
        # Draw arrow line
        cv2.arrowedLine(frame, (center_x, center_y), (end_x, end_y), (255, 0, 255), 4, tipLength=0.3)
        
        # Draw arrow label
        cv2.putText(frame, "Chase Direction", (end_x + 10, end_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
    
    def _draw_movement_status(self, frame):
        """Draw current movement status on frame"""
        # Get movement data from agent if available
        if hasattr(self.agent, 'current_movement_state'):
            state = self.agent.current_movement_state
            speed = getattr(self.agent, 'last_command_speed', 0.0)
            direction = getattr(self.agent, 'last_command_direction', 0.0)
            
            # Update movement data
            self.movement_data.update({
                'current_speed': speed,
                'current_direction': direction,
                'movement_state': state,
                'last_command_time': getattr(self.agent, 'last_command_time', 0.0)
            })
            
            # Draw status text
            status_text = f"State: {state} | Speed: {speed:.1f} | Dir: {direction:.1f}°"
            cv2.putText(frame, status_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    
    def run(self, host='0.0.0.0', port=5000, debug=False):
        """Run the web server"""
        print(f"Starting Ball Chaser Web Server on http://{host}:{port}")
        self.app.run(host=host, port=port, debug=debug, threaded=True)

# Standalone function to run the web server
def run_ball_chaser_server(camera, motor_controller, agent, host='0.0.0.0', port=5000):
    """Run the ball chaser web server"""
    server = BallChaserWebServer(camera, motor_controller, agent)
    server.run(host=host, port=port)

if __name__ == "__main__":
    print("Ball Chaser Web Server - Run this from bot.py instead")
