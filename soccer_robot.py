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
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        self.lower_orange = np.array([5, 150, 100])
        self.upper_orange = np.array([18, 255, 255])
        
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.motors = []
        self.motor_modes = []
        self.setup_motors()
        
        self.frame_center_x = 320
        self.frame_center_y = 240
        self.ball_center_x = 0
        self.ball_center_y = 0
        self.ball_radius = 0
        self.ball_detected = False
        
        self.max_speed = 8000000
        self.turn_speed = 4000000
        self.forward_speed = 6000000
        self.kp_turn = 0.8
        self.kp_forward = 0.6
        
    def setup_motors(self):
        motor_addresses = [10, 11]
        
        for i, addr in enumerate(motor_addresses):
            motor = PowerfulBLDCDriver(self.i2c, addr)
            
            if motor.get_firmware_version() != 3:
                print(f"error: motor {i} firmware version {motor.get_firmware_version()}")
                continue
                
            motor.set_current_limit_foc(65536)
            motor.set_id_pid_constants(1500, 200)
            motor.set_iq_pid_constants(1500, 200)
            motor.set_speed_pid_constants(4e-2, 4e-4, 3e-2)
            motor.set_position_pid_constants(275, 0, 0)
            motor.set_position_region_boundary(250000)
            motor.set_speed_limit(self.max_speed)
            
            motor.configure_operating_mode_and_sensor(15, 1)
            motor.configure_command_mode(15)
            motor.set_calibration_options(300, 2097152, 50000, 500000)
            
            motor.start_calibration()
            print(f"starting calibration of motor {i}")
            while not motor.is_calibration_finished():
                print(".", end="")
                sys.stdout.flush()
                time.sleep(0.5)
            print()
            print(f"elecangleoffset: {motor.get_calibration_ELECANGLEOFFSET()}")
            print(f"sincoscentre: {motor.get_calibration_SINCOSCENTRE()}")

            motor.configure_operating_mode_and_sensor(3, 1)
            motor.configure_command_mode(12)
            
            self.motors.append(motor)
            self.motor_modes.append(12)
            
        print(f"initialized {len(self.motors)} motors")
    
    def detect_ball(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        valid_contours = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if 500 < area < 50000:
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    if circularity > 0.6:
                        valid_contours.append(contour)
        
        if valid_contours:
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
        if not self.ball_detected or len(self.motors) < 2:
            return 0, 0
        
        error_x = self.ball_center_x - self.frame_center_x
        error_y = self.frame_center_y - self.ball_center_y
        
        error_x_norm = error_x / self.frame_center_x
        error_y_norm = error_y / self.frame_center_y
        
        left_speed = self.forward_speed + (error_x_norm * self.turn_speed * self.kp_turn)
        right_speed = self.forward_speed - (error_x_norm * self.turn_speed * self.kp_turn)
        
        if abs(error_y_norm) > 0.1:
            speed_adjustment = error_y_norm * self.forward_speed * self.kp_forward
            left_speed += speed_adjustment
            right_speed += speed_adjustment
        
        left_speed = np.clip(left_speed, -self.max_speed, self.max_speed)
        right_speed = np.clip(right_speed, -self.max_speed, self.max_speed)
        
        return int(left_speed), int(right_speed)
    
    def set_motor_speeds(self, left_speed, right_speed):
        if len(self.motors) >= 2:
            self.motors[0].set_speed(left_speed)
            self.motors[1].set_speed(right_speed)
    
    def stop_motors(self):
        for motor in self.motors:
            motor.set_speed(0)
    
    def run(self):
        print("soccer robot starting...")
        print("press 'q' to quit, 's' to stop motors")
        
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("failed to grab frame")
                    break
                
                ball_found, ball_center, ball_radius = self.detect_ball(frame)
                
                if ball_found:
                    left_speed, right_speed = self.calculate_motor_commands()
                    self.set_motor_speeds(left_speed, right_speed)
                    
                    cv2.circle(frame, ball_center, ball_radius, (0, 255, 0), 2)
                    cv2.circle(frame, ball_center, 5, (0, 0, 255), -1)
                    
                    cv2.circle(frame, (self.frame_center_x, self.frame_center_y), 10, (255, 0, 0), 2)
                    
                    cv2.putText(frame, f"l:{left_speed//1000}k r:{right_speed//1000}k", 
                              (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                else:
                    self.stop_motors()
                    cv2.putText(frame, "ball not found", (10, 30), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
                for motor in self.motors:
                    motor.update_quick_data_readout()
                
                cv2.imshow("soccer robot vision", frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    self.stop_motors()
                    print("motors stopped")
                
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("shutting down...")
        finally:
            self.stop_motors()
            self.cap.release()
            cv2.destroyAllWindows()
            print("robot shutdown complete")

def main():
    robot = SoccerRobot()
    robot.run()

if __name__ == "__main__":
    main() 