import cv2
import numpy as np
import time
import math
import sys
from steelbar_powerful_bldc_driver import PowerfulBLDCDriver
import board
import busio
from picamera2 import Picamera2
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

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
        self.max_speed = 150000000  # Maximum speed for all movements
        self.kp_turn = 2.0  # Turn sensitivity multiplier (increased for tighter turns)
        self.kp_forward = 0.8  # Forward movement sensitivity multiplier
        self.turn_threshold = 0.1  # Minimum error to start turning (reduces jitter)
        self.tight_turn_factor = 0.3  # Reduce forward speed during turns for tighter turning
        self.pure_turn_threshold = 0.4  # Error threshold for pure turning in place (no forward movement)
        self.nonlinear_turn_power = 0.5  # Power for nonlinear turning (0.5 = square root, 1.0 = linear)
        
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.motors = []
        self.motor_modes = []
        self.setup_motors()
        
        # Initialize BNO085 IMU for compass
        self.setup_imu()
        
        self.frame_center_x = 320
        self.frame_center_y = 240
        self.ball_center_x = 0
        self.ball_center_y = 0
        self.ball_radius = 0
        self.ball_detected = False
        
    def setup_motors(self, force_calibration=False):
        # 4 omniwheels: 27-back left, 28-back right, 30-front left, 26-front right
        motor_addresses = [27, 28, 30, 26]
        
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
                    time.sleep(0.001)
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
    
    def setup_imu(self):
        """Initialize the BNO085 IMU sensor for compass functionality"""
        try:
            # Initialize BNO085 using the standard I2C approach
            self.bno = BNO08X_I2C(self.i2c)
            
            # Enable the reports we need
            self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            
            # Initialize compass heading
            self.initial_heading = None
            self.current_heading = 0.0
            
            print("BNO085 IMU initialized successfully")
            
        except Exception as e:
            print(f"Failed to initialize BNO085 IMU: {e}")
            self.bno = None
    
    def get_compass_heading(self):
        """Get the current compass heading in degrees (0-360)"""
        if self.bno is None:
            return None
            
        try:
            # Get rotation vector (quaternion)
            quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
            
            # Convert quaternion to Euler angles
            # Calculate yaw (heading) from quaternion
            yaw = math.atan2(2.0 * (quat_real * quat_k + quat_i * quat_j),
                           1.0 - 2.0 * (quat_j * quat_j + quat_k * quat_k))
            
            # Convert from radians to degrees and normalize to 0-360
            heading = math.degrees(yaw)
            if heading < 0:
                heading += 360
            
            # Set initial heading on first reading
            if self.initial_heading is None:
                self.initial_heading = heading
            
            self.current_heading = heading
            return heading
            
        except Exception as e:
            print(f"Error reading compass heading: {e}")
            return None
    
    def get_relative_heading(self):
        """Get heading relative to initial orientation (0 = initial direction)"""
        if self.bno is None or self.initial_heading is None:
            return None
            
        current = self.get_compass_heading()
        if current is None:
            return None
            
        # Calculate relative heading
        relative = current - self.initial_heading
        if relative < 0:
            relative += 360
        elif relative >= 360:
            relative -= 360
            
        return relative
    
    def get_imu_data(self):
        """Get comprehensive IMU data including accelerometer, gyroscope, and magnetometer"""
        if self.bno is None:
            return None
            
        try:
            data = {
                'heading': self.get_compass_heading(),
                'relative_heading': self.get_relative_heading(),
                'acceleration': self.bno.acceleration,
                'gyro': self.bno.gyro,
                'magnetic': self.bno.magnetic
            }
            return data
        except Exception as e:
            print(f"Error reading IMU data: {e}")
            return None
    
    def detect_ball(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            print(f"Contour Area: {cv2.contourArea(contour)}")

        filtered_contours = [x for x in contours if cv2.contourArea(x) > 100 and cv2.contourArea(x) < 30000]

        contours = filtered_contours

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            
            self.ball_center_x = int(x)
            self.ball_center_y = int(y)
            self.ball_radius = int(radius)
            self.ball_detected = True
            
            return True, (self.ball_center_x, self.ball_center_y), self.ball_radius
        
        self.ball_detected = False
        return False, None, 0
    
    def calculate_motor_commands(self):
        if len(self.motors) < 4:
            return [0, 0, 0, 0]
        
        # Calculate error from ball position to center of frame
        if self.ball_detected:
            error_x = self.ball_center_x - self.frame_center_x
            error_y = self.frame_center_y - self.ball_center_y
            
            error_x_norm = error_x / self.frame_center_x
            error_y_norm = error_y / self.frame_center_y
        else:
            # Default values when ball not detected
            error_x_norm = 0
            error_y_norm = 0
        
        # CUSTOM OMNIWHEEL CONFIGURATION
        # Motors: [27-back left, 28-back right, 30-front left, 26-front right]
        # IMPORTANT: In this setup:
        # - Front-left (30) and back-left (27) motors are inverted due to hardware orientation
        # - For turning: left motors slow down, right motors speed up (to turn right)
        # - For turning: right motors slow down, left motors speed up (to turn left)

        # Calculate turning adjustment based on horizontal ball position
        # Apply turn threshold to reduce jitter and unnecessary small adjustments
        if abs(error_x_norm) < self.turn_threshold:
            turn_adjustment = 0
            is_turning = False
        else:
            # Positive error_x_norm means ball is to the right, so turn right
            # Negative error_x_norm means ball is to the left, so turn left
            
            # Use nonlinear turning response to reduce over-turning when ball is close to center
            # Apply a square root function to reduce sensitivity for small errors
            error_sign = 1 if error_x_norm > 0 else -1
            error_magnitude = abs(error_x_norm)
            
            # Nonlinear scaling: error^power for gentler response near center
            # This reduces over-turning when ball is close to center
            # 0.5 = square root (gentle), 1.0 = linear (aggressive)
            nonlinear_error = error_sign * (error_magnitude ** self.nonlinear_turn_power)
            
            turn_adjustment = nonlinear_error * self.max_speed * self.kp_turn
            is_turning = True

        # Base forward movement speed (based on max_speed)
        # Reduce forward speed during turns for tighter turning radius
        if is_turning:
            if abs(error_x_norm) > self.pure_turn_threshold:
                # Pure turning in place for large errors (minimal forward movement)
                forward_speed = self.max_speed * self.kp_forward * 0.1
            else:
                # Reduced forward speed for moderate turns
                forward_speed = self.max_speed * self.kp_forward * self.tight_turn_factor
        else:
            forward_speed = self.max_speed * self.kp_forward

        # Calculate individual motor speeds with turning
        # To turn right: slow down left motors, speed up right motors
        # To turn left: slow down right motors, speed up left motors
        
        # Back-left motor (27): forward movement (INVERTED) + turn adjustment (inverted for left motor)
        back_left_speed = -(forward_speed + turn_adjustment)

        # Back-right motor (28): forward movement - turn adjustment
        back_right_speed = forward_speed - turn_adjustment

        # Front-left motor (30): forward movement (INVERTED) + turn adjustment (inverted for left motor)
        front_left_speed = -(forward_speed + turn_adjustment)

        # Front-right motor (26): forward movement - turn adjustment
        front_right_speed = forward_speed - turn_adjustment
        
        # Clip speeds to max limits
        speeds = [back_left_speed, back_right_speed, front_left_speed, front_right_speed]
        speeds = [np.clip(speed, -self.max_speed, self.max_speed) for speed in speeds]
        
        return [int(speed) for speed in speeds]
    
    def set_motor_speeds(self, speeds):
        # speeds: [27-back left, 28-back right, 30-front left, 26-front right]
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
                
                ball_found, ball_center, ball_radius = self.detect_ball(frame)
                
                if ball_found:
                    # Move forward and turn towards ball when detected
                    speeds = self.calculate_motor_commands()
                    self.set_motor_speeds(speeds)
                    
                    # Calculate error for display
                    error_x = ball_center[0] - self.frame_center_x
                    error_x_norm = error_x / self.frame_center_x
                    
                    # Determine if turning and calculate turn adjustment for display
                    if abs(error_x_norm) < self.turn_threshold:
                        turn_adjustment = 0
                        is_turning = False
                        turn_mode = "STRAIGHT"
                    else:
                        # Use same nonlinear calculation as in motor commands
                        error_sign = 1 if error_x_norm > 0 else -1
                        error_magnitude = abs(error_x_norm)
                        nonlinear_error = error_sign * (error_magnitude ** self.nonlinear_turn_power)
                        turn_adjustment = nonlinear_error * self.max_speed * self.kp_turn
                        is_turning = True
                        if abs(error_x_norm) > self.pure_turn_threshold:
                            turn_mode = "PURE_TURN"
                        else:
                            turn_mode = "TIGHT_TURN"
                    
                    direction = "LEFT" if error_x_norm < 0 else "RIGHT" if error_x_norm > 0 else "CENTER"
                    
                    # Get compass heading for display
                    heading = self.get_compass_heading()
                    relative_heading = self.get_relative_heading()
                    heading_str = f"Heading: {heading:.1f}°" if heading is not None else "Heading: N/A"
                    relative_str = f"Rel: {relative_heading:.1f}°" if relative_heading is not None else "Rel: N/A"
                    
                    print(f"Ball at ({ball_center[0]}, {ball_center[1]}) - Error: {error_x_norm:.2f} ({direction}) - {turn_mode} - Turn: {turn_adjustment//1000}k - Speeds: BL:{speeds[0]//1000}k BR:{speeds[1]//1000}k FL:{speeds[2]//1000}k FR:{speeds[3]//1000}k - {heading_str} ({relative_str})")
                else:
                    # Stop motors when ball is not detected
                    self.stop_motors()
                    
                    # Get compass heading for display even when ball not found
                    heading = self.get_compass_heading()
                    relative_heading = self.get_relative_heading()
                    heading_str = f"Heading: {heading:.1f}°" if heading is not None else "Heading: N/A"
                    relative_str = f"Rel: {relative_heading:.1f}°" if relative_heading is not None else "Rel: N/A"
                    
                    print(f"Ball not found - motors stopped - {heading_str} ({relative_str})")
                
                for motor in self.motors:
                    motor.update_quick_data_readout()
                
                time.sleep(0.001)  # Reduced frequency for headless operation
                
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