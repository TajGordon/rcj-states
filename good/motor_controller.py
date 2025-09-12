
import time
import sys
import board
import busio
import math
import numpy as np
from steelbar_powerful_bldc_driver import PowerfulBLDCDriver
import config

class MotorController:
    def __init__(self, force_calibration=False, i2c=None):
        # Motor configuration
        self.max_speed = 30_000_000  # Maximum speed for all movements
        self.max_current = 4 * 65536  # 4A current limit (4 * 65536 for the driver)
        
        # Speed calculation: level * (1/8) * max_speed for fine-grained control
        # This allows for more precise speed control and easier scaling
        
        # Initialize I2C and motors
        if i2c is None:
            self.i2c = busio.I2C(board.SCL, board.SDA)
        else:
            self.i2c = i2c
        self.motors = []
        self.motor_modes = []
        self.current_heading = 0.0  # Current robot heading in radians
        self.current_x = 0.0  # Current robot X position (mm)
        self.current_y = 0.0  # Current robot Y position (mm)
        
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
            print(f"   🔧 SETTING MOTOR SPEEDS:")
            motor_names = ['BL', 'BR', 'FL', 'FR']
            for i, speed in enumerate(speeds):
                print(f"      {motor_names[i]} (motor {i}): {speed//1000}k")
                self.motors[i].set_speed(speed)
            print(f"   ✅ All motor speeds set")
        else:
            print(f"   ❌ ERROR: Not enough motors ({len(self.motors)}) or speeds ({len(speeds)})")
    
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
    def _calculate_speed(self, speed_level):
        """Calculate actual speed from speed level (-8.0 to 8.0, where negative = reverse)"""
        if speed_level < -8.0 or speed_level > 8.0:
            raise ValueError(f"Speed level must be between -8.0 and 8.0, got {speed_level}")
        if speed_level == 0:
            return 0
        
        # Apply smoothing for low speeds to reduce jittering
        if abs(speed_level) < 1.0:
            # For very low speeds, apply additional smoothing
            smoothed_level = speed_level * 0.8  # Reduce by 20% for smoother movement
        else:
            smoothed_level = speed_level
            
        return smoothed_level * (1.0 / 8.0) * self.max_speed

    def move_direction(self, direction_rad, speed_level, movement_type='relative'):
        """
        Move in a specific direction with speed level control
        
        Args:
            direction_rad: Direction in radians (0=forward, π/2=right, π=back, 3π/2=left)
            speed_level: Speed level -8.0 to 8.0 (negative = reverse, positive = forward, 0 = stop)
            movement_type: 'relative' (relative to robot) or 'absolute' (relative to field)
        """
        try:
            base_speed = self._calculate_speed(speed_level)
        except ValueError as e:
            print(f"Error: {e}")
            return
        
        # Normalize direction to 0-2π range
        direction_rad = direction_rad % (2 * math.pi)
        
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
        
        print(f"🎮 MOTOR CONTROLLER - move_direction:")
        print(f"   Input: direction={math.degrees(direction_rad):.1f}°, speed_level={speed_level}")
        print(f"   Calculated speeds: BL:{speeds[0]//1000}k BR:{speeds[1]//1000}k FL:{speeds[2]//1000}k FR:{speeds[3]//1000}k")
        print(f"   Forward component: {forward_speed//1000}k, Strafe component: {strafe_speed//1000}k")
    
    def move_with_rotation(self, direction_rad, move_speed_level, rotation_amount_rad, rotation_speed_level):
        """
        Move in a direction while simultaneously rotating
        
        Args:
            direction_rad: Direction to move in radians (0-2π, relative to robot's current heading)
            move_speed_level: Speed level for movement (-8.0 to 8.0, negative = reverse)
            rotation_amount_rad: Amount to rotate in radians (-π to π, negative=left, positive=right)
            rotation_speed_level: Speed level for rotation (-8.0 to 8.0, negative = reverse rotation)
        """
        try:
            move_speed = self._calculate_speed(move_speed_level)
            rotation_speed = self._calculate_speed(rotation_speed_level)
        except ValueError as e:
            print(f"Error: {e}")
            return
        
        # Normalize direction to 0-2π range
        direction_rad = direction_rad % (2 * math.pi)
        
        # Calculate movement components
        forward_speed = move_speed * math.cos(direction_rad)
        strafe_speed = move_speed * math.sin(direction_rad)
        
        # Calculate rotation component (positive = right turn, negative = left turn)
        # For rotation: left motors go one way, right motors go opposite way
        rotation_component = rotation_speed * (rotation_amount_rad / math.pi)  # Scale rotation amount
        
        # Combine movement and rotation
        # Back-left motor (27): forward + strafe + rotation (inverted)
        back_left_speed = -(forward_speed + strafe_speed + rotation_component)
        # Back-right motor (28): forward - strafe - rotation  
        back_right_speed = forward_speed - strafe_speed - rotation_component
        # Front-left motor (30): forward - strafe + rotation (inverted)
        front_left_speed = -(forward_speed - strafe_speed + rotation_component)
        # Front-right motor (26): forward + strafe - rotation
        front_right_speed = forward_speed + strafe_speed - rotation_component
        
        speeds = [int(back_left_speed), int(back_right_speed), int(front_left_speed), int(front_right_speed)]
        
        # Clip speeds to max limits
        speeds = [np.clip(speed, -self.max_speed, self.max_speed) for speed in speeds]
        
        self.set_motor_speeds(speeds)
        
        print(f"Moving {math.degrees(direction_rad):.1f}° at speed {move_speed_level} + rotating {math.degrees(rotation_amount_rad):.1f}° at speed {rotation_speed_level}")
        print(f"Speeds: BL:{speeds[0]//1000}k BR:{speeds[1]//1000}k FL:{speeds[2]//1000}k FR:{speeds[3]//1000}k")

    def turn_in_place(self, direction, speed_level):
        """
        Turn in place (no forward/backward movement)
        
        Args:
            direction: 'left' or 'right'
            speed_level: Speed level -8.0 to 8.0 (negative = reverse direction)
        """
        try:
            base_speed = self._calculate_speed(speed_level)
        except ValueError as e:
            print(f"Error: {e}")
            return
        
        if direction.lower() == 'left':
            # Turn left: left motors backward, right motors forward
            speeds = [int(base_speed), int(-base_speed), int(base_speed), int(-base_speed)]
        elif direction.lower() == 'right':
            # Turn right: left motors forward, right motors backward
            speeds = [int(-base_speed), int(base_speed), int(-base_speed), int(base_speed)]
        else:
            print(f"Invalid turn direction '{direction}'. Use 'left' or 'right'.")
            return
        
        self.set_motor_speeds(speeds)
        print(f"Turning {direction} at speed level {speed_level} - Speeds: BL:{speeds[0]//1000}k BR:{speeds[1]//1000}k FL:{speeds[2]//1000}k FR:{speeds[3]//1000}k")

    def move_to_orientation(self, target_heading_rad, speed_level, current_heading_rad=None):
        """
        Rotate to face a specific absolute orientation
        
        Args:
            target_heading_rad: Target heading in radians (0-2π, absolute field coordinates)
            speed_level: Speed level for rotation (-8.0 to 8.0, negative = reverse rotation)
            current_heading_rad: Current robot heading in radians (if None, uses stored heading)
        """
        if current_heading_rad is None:
            current_heading_rad = self.current_heading
        
        # Calculate shortest rotation path
        rotation_needed = target_heading_rad - current_heading_rad
        
        # Normalize to -π to π range for shortest path
        while rotation_needed > math.pi:
            rotation_needed -= 2 * math.pi
        while rotation_needed < -math.pi:
            rotation_needed += 2 * math.pi
        
        # Determine rotation direction
        if rotation_needed > 0:
            direction = 'right'
        else:
            direction = 'left'
            rotation_needed = abs(rotation_needed)
        
        # Scale rotation speed based on amount needed (more rotation = faster)
        rotation_speed = min(speed_level + 2, 8)  # Boost speed for orientation changes
        
        print(f"Rotating to {math.degrees(target_heading_rad):.1f}° (current: {math.degrees(current_heading_rad):.1f}°, need: {math.degrees(rotation_needed):.1f}° {direction})")
        self.turn_in_place(direction, rotation_speed)
        
        # Update stored heading
        self.current_heading = target_heading_rad

    def move_absolute_direction(self, target_direction_rad, speed_level, current_heading_rad=None):
        """
        Move in absolute direction relative to field coordinates
        
        Args:
            target_direction_rad: Target direction in radians (0-2π, field coordinates)
            speed_level: Speed level -8.0 to 8.0 (negative = reverse)
            current_heading_rad: Current robot heading in radians (if None, uses stored heading)
        """
        if current_heading_rad is None:
            current_heading_rad = self.current_heading
        
        # Calculate relative direction from robot's current heading
        relative_direction = target_direction_rad - current_heading_rad
        
        # Normalize to -π to π range
        while relative_direction > math.pi:
            relative_direction -= 2 * math.pi
        while relative_direction < -math.pi:
            relative_direction += 2 * math.pi
        
        # Convert to 0-2π range for move_direction
        if relative_direction < 0:
            relative_direction += 2 * math.pi
        
        self.move_direction(relative_direction, speed_level, 'relative')
        print(f"Absolute direction {math.degrees(target_direction_rad):.1f}° -> Relative direction {math.degrees(relative_direction):.1f}° (current heading: {math.degrees(current_heading_rad):.1f}°)")

    def move_to_position(self, target_x, target_y, speed_level, current_x=None, current_y=None, current_heading_rad=None):
        """
        Move to a specific position on the field
        
        Args:
            target_x: Target X coordinate (mm)
            target_y: Target Y coordinate (mm)
            speed_level: Speed level for movement (-8.0 to 8.0, negative = reverse)
            current_x: Current X position (if None, uses stored position)
            current_y: Current Y position (if None, uses stored position)
            current_heading_rad: Current robot heading in radians (if None, uses stored heading)
        """
        if current_x is None or current_y is None:
            print("Warning: Current position not provided, using relative movement")
            # For now, just move forward - in a real implementation, you'd need localization
            self.move_direction(0, speed_level)  # Move forward (0 radians)
            return
        
        if current_heading_rad is None:
            current_heading_rad = self.current_heading
        
        # Calculate distance and direction to target
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate absolute direction to target in radians
        target_direction_rad = math.atan2(dy, dx)
        if target_direction_rad < 0:
            target_direction_rad += 2 * math.pi
        
        print(f"Moving to position ({target_x}, {target_y}) - Distance: {distance:.1f}mm, Direction: {math.degrees(target_direction_rad):.1f}°")
        
        # First, rotate to face the target
        self.move_to_orientation(target_direction_rad, speed_level, current_heading_rad)
        
        # Then move forward (this would need to be called repeatedly until position is reached)
        # For now, just move forward - in a real implementation, you'd have a control loop
        self.move_direction(0, speed_level)  # Move forward (0 radians)
        
        # Update stored position (in a real implementation, this would come from localization)
        self.current_x = target_x
        self.current_y = target_y
        self.current_heading = target_direction_rad

    def move_absolute_direction_with_rotation(self, target_direction_rad, move_speed_level, target_heading_rad, rotation_speed_level, current_heading_rad=None):
        """
        Move in absolute direction while rotating to face a specific orientation
        
        Args:
            target_direction_rad: Target movement direction in radians (0-2π, field coordinates)
            move_speed_level: Speed level for movement (-8.0 to 8.0, negative = reverse)
            target_heading_rad: Target orientation to face in radians (0-2π)
            rotation_speed_level: Speed level for rotation (-8.0 to 8.0, negative = reverse rotation)
            current_heading_rad: Current robot heading in radians (if None, uses stored heading)
        """
        if current_heading_rad is None:
            current_heading_rad = self.current_heading
        
        # Calculate relative movement direction
        relative_direction = target_direction_rad - current_heading_rad
        while relative_direction > math.pi:
            relative_direction -= 2 * math.pi
        while relative_direction < -math.pi:
            relative_direction += 2 * math.pi
        if relative_direction < 0:
            relative_direction += 2 * math.pi
        
        # Calculate rotation needed
        rotation_needed = target_heading_rad - current_heading_rad
        while rotation_needed > math.pi:
            rotation_needed -= 2 * math.pi
        while rotation_needed < -math.pi:
            rotation_needed += 2 * math.pi
        
        print(f"Moving {math.degrees(target_direction_rad):.1f}° (relative: {math.degrees(relative_direction):.1f}°) while rotating to {math.degrees(target_heading_rad):.1f}° (need: {math.degrees(rotation_needed):.1f}°)")
        
        # Use the combined movement and rotation function
        self.move_with_rotation(relative_direction, move_speed_level, rotation_needed, rotation_speed_level)

    def set_heading(self, heading_rad):
        """Set the current robot heading for relative movement calculations"""
        self.current_heading = heading_rad % (2 * math.pi)
        print(f"Robot heading set to {math.degrees(self.current_heading):.1f}°")
    
    def set_position(self, x, y):
        """Set the current robot position for navigation calculations"""
        self.current_x = x
        self.current_y = y
        print(f"Robot position set to ({x}, {y})")

    # CONVENIENCE FUNCTIONS FOR DEGREES (backward compatibility)
    def move_direction_degrees(self, direction_degrees, speed_level, movement_type='relative'):
        """Move in a specific direction using degrees (convenience function)"""
        direction_rad = math.radians(direction_degrees)
        self.move_direction(direction_rad, speed_level, movement_type)
    
    def move_with_rotation_degrees(self, direction_degrees, move_speed_level, rotation_amount_degrees, rotation_speed_level):
        """Move with rotation using degrees (convenience function)"""
        direction_rad = math.radians(direction_degrees)
        rotation_amount_rad = math.radians(rotation_amount_degrees)
        self.move_with_rotation(direction_rad, move_speed_level, rotation_amount_rad, rotation_speed_level)
    
    def move_to_orientation_degrees(self, target_heading_degrees, speed_level, current_heading_degrees=None):
        """Rotate to face a specific orientation using degrees (convenience function)"""
        target_heading_rad = math.radians(target_heading_degrees)
        current_heading_rad = math.radians(current_heading_degrees) if current_heading_degrees is not None else None
        self.move_to_orientation(target_heading_rad, speed_level, current_heading_rad)
    
    def move_absolute_direction_degrees(self, target_direction_degrees, speed_level, current_heading_degrees=None):
        """Move in absolute direction using degrees (convenience function)"""
        target_direction_rad = math.radians(target_direction_degrees)
        current_heading_rad = math.radians(current_heading_degrees) if current_heading_degrees is not None else None
        self.move_absolute_direction(target_direction_rad, speed_level, current_heading_rad)
    
    def move_absolute_direction_with_rotation_degrees(self, target_direction_degrees, move_speed_level, target_heading_degrees, rotation_speed_level, current_heading_degrees=None):
        """Move in absolute direction with rotation using degrees (convenience function)"""
        target_direction_rad = math.radians(target_direction_degrees)
        target_heading_rad = math.radians(target_heading_degrees)
        current_heading_rad = math.radians(current_heading_degrees) if current_heading_degrees is not None else None
        self.move_absolute_direction_with_rotation(target_direction_rad, move_speed_level, target_heading_rad, rotation_speed_level, current_heading_rad)
    
    def set_heading_degrees(self, heading_degrees):
        """Set the current robot heading using degrees (convenience function)"""
        heading_rad = math.radians(heading_degrees)
        self.set_heading(heading_rad)


    def get_speed_level_info(self):
        """Get information about available speed levels"""
        print("Available speed levels:")
        print("  Range: -8.0 to 8.0 (supports fractional values like 0.3, 2.5, etc.)")
        print("  Reverse speeds (negative):")
        for level in range(-8, 0):
            multiplier = abs(level) * (1.0 / 8.0)
            speed_value = int(self.max_speed * multiplier)
            print(f"    Level {level}: -{multiplier*100:.1f}% of max speed (-{speed_value//1000}k)")
        print("  Stop:")
        print(f"    Level 0: 0% of max speed (0k)")
        print("  Forward speeds (positive):")
        for level in range(1, 9):
            multiplier = level * (1.0 / 8.0)
            speed_value = int(self.max_speed * multiplier)
            print(f"    Level {level}: +{multiplier*100:.1f}% of max speed (+{speed_value//1000}k)")
        print("  Examples of fractional speeds:")
        examples = [0.3, 1.5, 2.7, 4.2, 6.8]
        for example in examples:
            multiplier = example * (1.0 / 8.0)
            speed_value = int(self.max_speed * multiplier)
            print(f"    Level {example}: +{multiplier*100:.1f}% of max speed (+{speed_value//1000}k)")
        print(f"  Max current limit: 4A")
        print(f"  Max speed: ±{self.max_speed//1000}k")

    def update_motor_data(self):
        """Update motor data readouts (call this regularly in main loop)"""
        for motor in self.motors:
            motor.update_quick_data_readout()
    
    def are_motors_ready(self):
        """Check if all motors are properly initialized and ready"""
        return len(self.motors) >= 4 and all(motor is not None for motor in self.motors)

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