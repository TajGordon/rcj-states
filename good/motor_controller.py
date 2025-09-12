
import time
import sys
import board
import busio
import math
import numpy as np
from steelbar_powerful_bldc_driver import PowerfulBLDCDriver
import config

class MotorController:
    def __init__(self, force_calibration=False):
        # Motor configuration
        self.max_speed = 50_000_000  # Maximum speed for all movements
        self.max_current = 4 * 65536  # 4A current limit (4 * 65536 for the driver)
        
        # Speed levels (1-8) with corresponding multipliers
        self.speed_levels = {
            1: 0.125/2,  # 12.5% of max speed
            2: 0.25/2,   # 25% of max speed
            3: 0.375/2,  # 37.5% of max speed
            4: 0.5/2,    # 50% of max speed
            5: 0.625/2,  # 62.5% of max speed
            6: 0.75/2,   # 75% of max speed
            7: 0.875/2,  # 87.5% of max speed
            8: 1.0/2     # 100% of max speed
        }
        
        # Initialize I2C and motors
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.motors = []
        self.motor_modes = []
        self.current_heading = 0.0  # Current robot heading for relative movement
        
        # Initialize motors
        self.setup_motors(force_calibration)
        
    def setup_motors(self, force_calibration=False):
        """Initialize all motors with proper configuration"""
        print("Setting up omniwheel motors...")
        
        for i, addr in enumerate(config.motor_addresses):
            print(f"Initializing motor {i} at address {addr}...")
            motor = PowerfulBLDCDriver(self.i2c, addr)
            
            if motor.get_firmware_version() != 3:
                print(f"ERROR: Motor {i} firmware version {motor.get_firmware_version()} (expected 3)")
                continue
                
            # Configure motor parameters with 4A current limit
            motor.set_current_limit_foc(self.max_current)
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
                # Skip calibration - use previously calibrated values
                print(f"Motor {i}: Using previously calibrated values")
                print(f"  elecangleoffset: {motor.get_calibration_ELECANGLEOFFSET()}")
                print(f"  sincoscentre: {motor.get_calibration_SINCOSCENTRE()}")

            motor.configure_operating_mode_and_sensor(3, 1)
            motor.configure_command_mode(12)
            
            self.motors.append(motor)
            self.motor_modes.append(12)
            
        print(f"Successfully initialized {len(self.motors)} motors")
        
        if len(self.motors) < 4:
            print("ERROR: Not all motors initialized. Check connections and addresses.")
            return False
        return True

    # DIRECT MOTOR CONTROL FUNCTIONS
    def set_motor_speeds(self, speeds):
        """Direct motor speed control - speeds: [back_left, back_right, front_left, front_right]"""
        if len(self.motors) >= 4 and len(speeds) >= 4:
            for i, speed in enumerate(speeds):
                self.motors[i].set_speed(speed)
    
    def set_individual_motor_speed(self, motor_index, speed):
        """Set speed for individual motor (0=back_left, 1=back_right, 2=front_left, 3=front_right)"""
        if 0 <= motor_index < len(self.motors):
            self.motors[motor_index].set_speed(speed)
    
    def stop_motors(self):
        """Stop all motors immediately"""
        for motor in self.motors:
            motor.set_speed(0)
    
    def stop_individual_motor(self, motor_index):
        """Stop individual motor"""
        if 0 <= motor_index < len(self.motors):
            self.motors[motor_index].set_speed(0)

    # HIGH-LEVEL MOVEMENT FUNCTIONS
    def move_direction(self, direction_degrees, speed_level, movement_type='relative'):
        """
        Move in a specific direction with speed level control
        
        Args:
            direction_degrees: Direction in degrees (0-360, 0=forward, 90=right, 180=back, 270=left)
            speed_level: Speed level 1-8 (1=slowest, 8=fastest)
            movement_type: 'relative' (relative to robot) or 'absolute' (relative to field)
        """
        if speed_level not in self.speed_levels:
            print(f"Invalid speed level {speed_level}. Must be 1-8.")
            return
        
        # Normalize direction to 0-360 range
        direction_degrees = direction_degrees % 360
        
        # Convert to radians
        direction_rad = math.radians(direction_degrees)
        
        # Calculate base speed from level
        base_speed = self.max_speed * self.speed_levels[speed_level]
        
        # Calculate forward and strafe components
        forward_speed = base_speed * math.cos(direction_rad)
        strafe_speed = base_speed * math.sin(direction_rad)
        
        # For absolute movement, adjust for robot's current heading
        if movement_type == 'absolute':
            # This would require localization data - for now, use relative
            # TODO: Integrate with localization system
            pass
        
        # Calculate individual motor speeds for omniwheel movement
        # Motors: [back_left(27), back_right(28), front_left(30), front_right(26)]
        # Note: Front-left (30) and back-left (27) motors are inverted due to hardware orientation
        
        # Back-left motor (27): forward + strafe (inverted)
        back_left_speed = -(forward_speed + strafe_speed)
        # Back-right motor (28): forward - strafe  
        back_right_speed = forward_speed - strafe_speed
        # Front-left motor (30): forward - strafe (inverted)
        front_left_speed = -(forward_speed - strafe_speed)
        # Front-right motor (26): forward + strafe
        front_right_speed = forward_speed + strafe_speed
        
        speeds = [int(back_left_speed), int(back_right_speed), int(front_left_speed), int(front_right_speed)]
        
        # Clip speeds to max limits
        speeds = [np.clip(speed, -self.max_speed, self.max_speed) for speed in speeds]
        
        self.set_motor_speeds(speeds)
        
        print(f"Moving {direction_degrees}° at speed level {speed_level} ({movement_type}) - Speeds: BL:{speeds[0]//1000}k BR:{speeds[1]//1000}k FL:{speeds[2]//1000}k FR:{speeds[3]//1000}k")

    def move_forward(self, speed_level):
        """Move forward at specified speed level"""
        self.move_direction(0, speed_level)
    
    def move_backward(self, speed_level):
        """Move backward at specified speed level"""
        self.move_direction(180, speed_level)
    
    def move_left(self, speed_level):
        """Move left at specified speed level"""
        self.move_direction(270, speed_level)
    
    def move_right(self, speed_level):
        """Move right at specified speed level"""
        self.move_direction(90, speed_level)
    
    def turn_in_place(self, direction, speed_level):
        """
        Turn in place (no forward/backward movement)
        
        Args:
            direction: 'left' or 'right'
            speed_level: Speed level 1-8
        """
        if speed_level not in self.speed_levels:
            print(f"Invalid speed level {speed_level}. Must be 1-8.")
            return
        
        base_speed = self.max_speed * self.speed_levels[speed_level]
        
        if direction.lower() == 'left':
            # Turn left: left motors backward, right motors forward
            speeds = [base_speed, -base_speed, base_speed, -base_speed]
        elif direction.lower() == 'right':
            # Turn right: left motors forward, right motors backward
            speeds = [-base_speed, base_speed, -base_speed, base_speed]
        else:
            print(f"Invalid turn direction '{direction}'. Use 'left' or 'right'.")
            return
        
        self.set_motor_speeds(speeds)
        print(f"Turning {direction} at speed level {speed_level} - Speeds: BL:{speeds[0]//1000}k BR:{speeds[1]//1000}k FL:{speeds[2]//1000}k FR:{speeds[3]//1000}k")

    def move_absolute_direction(self, target_direction_degrees, speed_level, current_heading=None):
        """
        Move in absolute direction relative to field coordinates
        
        Args:
            target_direction_degrees: Target direction in field coordinates (0-360)
            speed_level: Speed level 1-8
            current_heading: Current robot heading (if None, uses relative movement)
        """
        if current_heading is None:
            print("No current heading provided, using relative movement")
            self.move_direction(target_direction_degrees, speed_level, 'relative')
            return
        
        # Calculate relative direction from robot's current heading
        relative_direction = target_direction_degrees - current_heading
        
        # Normalize to -180 to 180 range
        while relative_direction > 180:
            relative_direction -= 360
        while relative_direction < -180:
            relative_direction += 360
        
        # Convert to 0-360 range for move_direction
        if relative_direction < 0:
            relative_direction += 360
        
        self.move_direction(relative_direction, speed_level, 'relative')
        print(f"Absolute direction {target_direction_degrees}° -> Relative direction {relative_direction}° (current heading: {current_heading}°)")

    def set_heading(self, heading_degrees):
        """Set the current robot heading for relative movement calculations"""
        self.current_heading = heading_degrees % 360
        print(f"Robot heading set to {self.current_heading}°")

    def get_speed_level_info(self):
        """Get information about available speed levels"""
        print("Available speed levels:")
        for level, multiplier in self.speed_levels.items():
            speed_value = int(self.max_speed * multiplier)
            print(f"  Level {level}: {multiplier*100:.1f}% of max speed ({speed_value//1000}k)")
        print(f"  Max current limit: 4A")
        print(f"  Max speed: {self.max_speed//1000}k")

    def update_motor_data(self):
        """Update motor data readouts (call this regularly in main loop)"""
        for motor in self.motors:
            motor.update_quick_data_readout()

    def run(self):
        """Main control loop - override this for your specific application"""
        print("Motor controller running...")
        print("Press Ctrl+C to quit")
        
        try:
            while True:
                self.update_motor_data()
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("Shutting down motor controller...")
        finally:
            self.stop_motors()
            print("Motor controller stopped")