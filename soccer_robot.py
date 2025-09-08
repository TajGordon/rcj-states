import cv2
import numpy as np
import time
import math
import sys
from steelbar_powerful_bldc_driver import PowerfulBLDCDriver
import board
import busio
from picamera2 import Picamera2

class SoccerRobot:
    def __init__(self):
        # Initialize Pi Camera
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_video_configuration(
            main={"size": (640, 480), "format": "RGB888"}
        ))
        self.picam2.start()
        
        # Create a dummy cap object for compatibility with existing code
        self.cap = None
        
        self.lower_orange = np.array([0,132,61]) #[0,132,61]
        self.upper_orange = np.array([14,255,255]) #[14,255,255]
        
        # Define speed parameters before setup_motors() is called
        self.max_speed = 8000000
        self.turn_speed = 4000000
        self.forward_speed = 6000000
        self.kp_turn = 0.8
        self.kp_forward = 0.6
        
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
        
    def setup_motors(self, force_calibration=False):
        # 4 omniwheels: 26-back, 27-right, 28-front, 30-left
        motor_addresses = [26, 27, 28, 30]
        
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
            
            if force_calibration:
                # Run full calibration
                motor.start_calibration()
                print(f"Starting calibration of motor {i}")
                while not motor.is_calibration_finished():
                    print(".", end="")
                    sys.stdout.flush()
                    time.sleep(0.5)
                print()
                print(f"  elecangleoffset: {motor.get_calibration_ELECANGLEOFFSET()}")
                print(f"  sincoscentre: {motor.get_calibration_SINCOSCENTRE()}")
            else:
                # Skip calibration - values are stored internally by the motor driver
                # The motor driver will use the previously calibrated values automatically
                print(f"Motor {i}: Using previously calibrated values (stored internally)")
                print(f"  elecangleoffset: {motor.get_calibration_ELECANGLEOFFSET()}")
                print(f"  sincoscentre: {motor.get_calibration_SINCOSCENTRE()}")

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
            if 0 < area < 5000000:
                print(f"Contour area: {area}")
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
        if not self.ball_detected or len(self.motors) < 4:
            return [0, 0, 0, 0]
        
        error_x = self.ball_center_x - self.frame_center_x
        error_y = self.frame_center_y - self.ball_center_y
        
        error_x_norm = error_x / self.frame_center_x
        error_y_norm = error_y / self.frame_center_y
        
        # Omniwheel movement calculation
        # Motors: [back(26), right(27), front(28), left(30)]
        # For omniwheels: forward/backward affects front/back, left/right affects left/right
        # Turning affects all wheels in opposite directions
        
        # Base forward movement
        forward_speed = error_y_norm * self.forward_speed * self.kp_forward
        
        # Sideways movement (strafing)
        strafe_speed = error_x_norm * self.forward_speed * self.kp_forward
        
        # Turning movement
        turn_speed = error_x_norm * self.turn_speed * self.kp_turn
        
        # Calculate individual motor speeds
        # Back motor (26): forward + turn
        back_speed = forward_speed + turn_speed
        
        # Right motor (27): strafe + turn  
        right_speed = strafe_speed + turn_speed
        
        # Front motor (28): forward - turn
        front_speed = forward_speed - turn_speed
        
        # Left motor (30): strafe - turn
        left_speed = strafe_speed - turn_speed
        
        # Clip speeds to max limits
        speeds = [back_speed, right_speed, front_speed, left_speed]
        speeds = [np.clip(speed, -self.max_speed, self.max_speed) for speed in speeds]
        
        return [int(speed) for speed in speeds]
    
    def set_motor_speeds(self, speeds):
        # speeds: [back(26), right(27), front(28), left(30)]
        if len(self.motors) >= 4 and len(speeds) >= 4:
            for i, speed in enumerate(speeds):
                self.motors[i].set_speed(speed)
    
    def stop_motors(self):
        for motor in self.motors:
            motor.set_speed(0)
    
    def run(self):
        print("soccer robot starting...")
        print("press Ctrl+C to quit")
        
        try:
            while True:
                # Capture frame from Pi Camera
                frame = self.picam2.capture_array()
                # Convert from RGB to BGR for OpenCV
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                
                ball_found, ball_center, ball_radius = self.detect_ball(frame)
                
                if ball_found:
                    speeds = self.calculate_motor_commands()
                    self.set_motor_speeds(speeds)
                    print(f"Ball found at ({ball_center[0]}, {ball_center[1]}) - speeds: B:{speeds[0]//1000}k R:{speeds[1]//1000}k F:{speeds[2]//1000}k L:{speeds[3]//1000}k")
                else:
                    self.stop_motors()
                    print("Ball not found - motors stopped")
                
                for motor in self.motors:
                    motor.update_quick_data_readout()
                
                time.sleep(0.1)  # Reduced frequency for headless operation
                
        except KeyboardInterrupt:
            print("shutting down...")
        finally:
            self.stop_motors()
            self.picam2.stop()
            print("robot shutdown complete")

def main():
    # Set force_calibration=True if you need to recalibrate motors
    # This is needed for first-time setup or after hardware changes
    force_calibration = False  # Change to True to force calibration
    
    robot = SoccerRobot()
    if force_calibration:
        print("WARNING: Force calibration is enabled!")
        print("This will recalibrate all motors and may take several minutes.")
        print("Only do this if motors aren't working properly or after hardware changes.")
        input("Press Enter to continue with calibration, or Ctrl+C to cancel...")
        robot.setup_motors(force_calibration=True)
    
    robot.run()

if __name__ == "__main__":
    main() 