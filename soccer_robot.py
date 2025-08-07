import cv2
import numpy as np
import time
import math
import sys
from steelbar_powerful_bldc_driver import PowerfulBLDCDriver
import board
import busio

class SoccerRobot:
    def __init__(self):
        # Camera setup
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Orange ball detection parameters (optimized for RoboCup)
        self.lower_orange = np.array([5, 150, 100])  # More robust orange detection
        self.upper_orange = np.array([18, 255, 255])
        
        # Motor control setup
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.motors = []
        self.motor_modes = []
        self.setup_motors()
        
        # Robot parameters
        self.frame_center_x = 320
        self.frame_center_y = 240
        self.ball_center_x = 0
        self.ball_center_y = 0
        self.ball_radius = 0
        self.ball_detected = False
        
        # Control parameters
        self.max_speed = 8000000  # Reduced for precision
        self.turn_speed = 4000000
        self.forward_speed = 6000000
        self.kp_turn = 0.8  # Proportional gain for turning
        self.kp_forward = 0.6  # Proportional gain for forward movement
        
    def setup_motors(self):
        """Initialize motor drivers for differential drive"""
        # Assuming 2 motors: left (index 0) and right (index 1)
        motor_addresses = [10, 11]  # Adjust based on your setup
        
        for i, addr in enumerate(motor_addresses):
            motor = PowerfulBLDCDriver(self.i2c, addr)
            
            # Verify firmware version
            if motor.get_firmware_version() != 3:
                print(f"Error: Motor {i} firmware version {motor.get_firmware_version()}")
                continue
                
            # Configure motor
            motor.set_current_limit_foc(65536)  # 1A current limit
            motor.set_id_pid_constants(1500, 200)
            motor.set_iq_pid_constants(1500, 200)
            motor.set_speed_pid_constants(4e-2, 4e-4, 3e-2)  # Constants valid for FOC and Robomaster M2006 P36 motor only
            motor.set_position_pid_constants(275, 0, 0)
            motor.set_position_region_boundary(250000)
            motor.set_speed_limit(self.max_speed)
            
            # CALIBRATION STEP - Configure calibration mode
            motor.configure_operating_mode_and_sensor(15, 1)  # configure calibration mode and sin/cos encoder
            motor.configure_command_mode(15)  # configure calibration mode
            motor.set_calibration_options(300, 2097152, 50000, 500000)  # set calibration voltage to 300/3399*vcc volts, speed to 2097152/65536 elecangle/s, settling time to 50000/50000 seconds, calibration time to 500000/50000 seconds
            
            # Start calibration
            motor.start_calibration()  # start the calibration
            print(f"Starting calibration of motor {i}")
            while not motor.is_calibration_finished():  # wait for the calibration to finish
                print(".", end="")
                sys.stdout.flush()
                time.sleep(0.5)
            print()  # print out the calibration results
            print(f"ELECANGLEOFFSET: {motor.get_calibration_ELECANGLEOFFSET()}")
            print(f"SINCOSCENTRE: {motor.get_calibration_SINCOSCENTRE()}")

            # Configure for normal operation
            motor.configure_operating_mode_and_sensor(3, 1)  # configure FOC mode and sin/cos encoder
            motor.configure_command_mode(12)  # configure speed command mode
            
            self.motors.append(motor)
            self.motor_modes.append(12)
            
        print(f"Initialized {len(self.motors)} motors")
    
    def detect_ball(self, frame):
        """Detect orange ball with improved filtering"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        
        # Morphological operations to reduce noise
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours by area and circularity
        valid_contours = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if 500 < area < 50000:  # Adjusted for RoboCup ball size
                # Check circularity
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    if circularity > 0.6:  # Ball should be roughly circular
                        valid_contours.append(contour)
        
        if valid_contours:
            # Get the largest valid contour
            largest_contour = max(valid_contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            
            self.ball_center_x = int(x)
            self.ball_center_y = int(y)
            self.ball_radius = int(radius)
            self.ball_detected = True
            
            return True, (self.ball_center_x, self.ball_center_y), self.ball_radius
        
        self.ball_detected = False
        return False, None, 0
    
    def calculate_motor_commands(self):
        """Calculate motor speeds based on ball position"""
        if not self.ball_detected or len(self.motors) < 2:
            return 0, 0
        
        # Calculate error from center
        error_x = self.ball_center_x - self.frame_center_x
        error_y = self.frame_center_y - self.ball_center_y  # Inverted for intuitive control
        
        # Normalize errors
        error_x_norm = error_x / self.frame_center_x
        error_y_norm = error_y / self.frame_center_y
        
        # Calculate motor speeds for differential drive
        # Left motor speed
        left_speed = self.forward_speed + (error_x_norm * self.turn_speed * self.kp_turn)
        # Right motor speed  
        right_speed = self.forward_speed - (error_x_norm * self.turn_speed * self.kp_turn)
        
        # Adjust based on ball distance (Y position)
        if abs(error_y_norm) > 0.1:  # Ball is too far or too close
            speed_adjustment = error_y_norm * self.forward_speed * self.kp_forward
            left_speed += speed_adjustment
            right_speed += speed_adjustment
        
        # Clamp speeds
        left_speed = np.clip(left_speed, -self.max_speed, self.max_speed)
        right_speed = np.clip(right_speed, -self.max_speed, self.max_speed)
        
        return int(left_speed), int(right_speed)
    
    def set_motor_speeds(self, left_speed, right_speed):
        """Set motor speeds"""
        if len(self.motors) >= 2:
            self.motors[0].set_speed(left_speed)   # Left motor
            self.motors[1].set_speed(right_speed)  # Right motor
    
    def stop_motors(self):
        """Stop all motors"""
        for motor in self.motors:
            motor.set_speed(0)
    
    def run(self):
        """Main robot control loop"""
        print("Soccer Robot Starting...")
        print("Press 'q' to quit, 's' to stop motors")
        
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("Failed to grab frame")
                    break
                
                # Detect ball
                ball_found, ball_center, ball_radius = self.detect_ball(frame)
                
                # Calculate and set motor commands
                if ball_found:
                    left_speed, right_speed = self.calculate_motor_commands()
                    self.set_motor_speeds(left_speed, right_speed)
                    
                    # Draw ball detection
                    cv2.circle(frame, ball_center, ball_radius, (0, 255, 0), 2)
                    cv2.circle(frame, ball_center, 5, (0, 0, 255), -1)
                    
                    # Draw target center
                    cv2.circle(frame, (self.frame_center_x, self.frame_center_y), 10, (255, 0, 0), 2)
                    
                    # Display speeds
                    cv2.putText(frame, f"L:{left_speed//1000}k R:{right_speed//1000}k", 
                              (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                else:
                    # Ball not found - stop motors
                    self.stop_motors()
                    cv2.putText(frame, "BALL NOT FOUND", (10, 30), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
                # Update motor data
                for motor in self.motors:
                    motor.update_quick_data_readout()
                
                # Display frame
                cv2.imshow("Soccer Robot Vision", frame)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    self.stop_motors()
                    print("Motors stopped")
                
                # Small delay for stability
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("Shutting down...")
        finally:
            self.stop_motors()
            self.cap.release()
            cv2.destroyAllWindows()
            print("Robot shutdown complete")

def main():
    robot = SoccerRobot()
    robot.run()

if __name__ == "__main__":
    main() 