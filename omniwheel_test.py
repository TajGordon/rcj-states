import time
import sys
import board
import busio
from steelbar_powerful_bldc_driver import PowerfulBLDCDriver

class OmniwheelTest:
    def __init__(self):
        # Define speed parameters (same as soccer robot)
        self.max_speed = 150000000
        self.test_speed = 10000000  # Moderate speed for testing
        
        # Initialize I2C and motors
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.motors = []
        self.motor_modes = []
        self.setup_motors()
        
    def setup_motors(self, force_calibration=False):
        # 4 omniwheels: 27-back left, 28-back right, 30-front left, 26-front right
        motor_addresses = [27, 28, 30, 26]
        
        print("Setting up omniwheel motors...")
        
        for i, addr in enumerate(motor_addresses):
            print(f"Initializing motor {i} at address {addr}...")
            motor = PowerfulBLDCDriver(self.i2c, addr)
            
            if motor.get_firmware_version() != 3:
                print(f"ERROR: Motor {i} firmware version {motor.get_firmware_version()} (expected 3)")
                continue
                
            # Configure motor parameters (same as soccer robot)
            motor.set_current_limit_foc(4*65536)
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
    
    def set_motor_speeds(self, speeds):
        # speeds: [back_left(27), back_right(28), front_left(30), front_right(26)]
        if len(self.motors) >= 4 and len(speeds) >= 4:
            for i, speed in enumerate(speeds):
                self.motors[i].set_speed(speed)
    
    def stop_motors(self):
        print("Stopping all motors...")
        for motor in self.motors:
            motor.set_speed(0)
        time.sleep(0.5)  # Give motors time to stop
    
    def test_forward_movement(self, duration=3):
        """Test forward movement - all wheels rotate in same direction"""
        print(f"\n=== Testing FORWARD movement for {duration} seconds ===")
        print("All wheels should rotate forward (same direction)")
        
        # All motors move forward at same speed
        # Note: Front-left (30) and back-left (27) motors are inverted due to hardware orientation
        forward_speeds = [-self.test_speed, self.test_speed, -self.test_speed, self.test_speed]
        self.set_motor_speeds(forward_speeds)
        
        print(f"Motor speeds: Back_L:{forward_speeds[0]//1000}k, Back_R:{forward_speeds[1]//1000}k, Front_L:{forward_speeds[2]//1000}k, Front_R:{forward_speeds[3]//1000}k")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            for motor in self.motors:
                motor.update_quick_data_readout()
            time.sleep(0.01)
        
        self.stop_motors()
        print("Forward test completed")
    
    def test_backward_movement(self, duration=3):
        """Test backward movement - all wheels rotate in opposite direction"""
        print(f"\n=== Testing BACKWARD movement for {duration} seconds ===")
        print("All wheels should rotate backward (opposite direction)")
        
        # All motors move backward at same speed
        # Note: Front-left (30) and back-left (27) motors are inverted due to hardware orientation
        backward_speeds = [self.test_speed, -self.test_speed, self.test_speed, -self.test_speed]
        self.set_motor_speeds(backward_speeds)
        
        print(f"Motor speeds: Back_L:{backward_speeds[0]//1000}k, Back_R:{backward_speeds[1]//1000}k, Front_L:{backward_speeds[2]//1000}k, Front_R:{backward_speeds[3]//1000}k")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            for motor in self.motors:
                motor.update_quick_data_readout()
            time.sleep(0.01)
        
        self.stop_motors()
        print("Backward test completed")
    
    def test_individual_motors(self, duration=2):
        """Test each motor individually"""
        print(f"\n=== Testing INDIVIDUAL motors for {duration} seconds each ===")
        
        motor_names = ["Back_Left (27)", "Back_Right (28)", "Front_Left (30)", "Front_Right (26)"]
        
        for i, name in enumerate(motor_names):
            print(f"\nTesting {name}...")
            
            # Stop all motors first
            self.stop_motors()
            time.sleep(0.5)
            
            # Test forward
            speeds = [0, 0, 0, 0]
            speeds[i] = self.test_speed
            self.set_motor_speeds(speeds)
            print(f"  Forward: {name} at {self.test_speed//1000}k")
            
            start_time = time.time()
            while time.time() - start_time < duration:
                self.motors[i].update_quick_data_readout()
                time.sleep(0.01)
            
            # Test backward
            speeds[i] = -self.test_speed
            self.set_motor_speeds(speeds)
            print(f"  Backward: {name} at {-self.test_speed//1000}k")
            
            start_time = time.time()
            while time.time() - start_time < duration:
                self.motors[i].update_quick_data_readout()
                time.sleep(0.01)
            
            self.stop_motors()
            time.sleep(0.5)
        
        print("Individual motor tests completed")
    
    def test_diagonal_movement(self, duration=3):
        """Test diagonal movement like in soccer robot"""
        print(f"\n=== Testing DIAGONAL movement for {duration} seconds ===")
        print("Testing forward + strafe movement (like soccer robot)")
        
        # Forward + strafe movement (same calculation as soccer robot)
        forward_speed = self.test_speed
        strafe_speed = self.test_speed // 2  # Half speed for strafe
        
        # Calculate individual motor speeds for diagonal movement
        # Note: Front-left (30) and back-left (27) motors are inverted due to hardware orientation
        # Back-left motor (27): forward + strafe (inverted)
        back_left_speed = -(forward_speed + strafe_speed)
        # Back-right motor (28): forward - strafe  
        back_right_speed = forward_speed - strafe_speed
        # Front-left motor (30): forward - strafe (inverted)
        front_left_speed = -(forward_speed - strafe_speed)
        # Front-right motor (26): forward + strafe
        front_right_speed = forward_speed + strafe_speed
        
        diagonal_speeds = [back_left_speed, back_right_speed, front_left_speed, front_right_speed]
        self.set_motor_speeds(diagonal_speeds)
        
        print(f"Diagonal speeds: Back_L:{back_left_speed//1000}k, Back_R:{back_right_speed//1000}k, Front_L:{front_left_speed//1000}k, Front_R:{front_right_speed//1000}k")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            for motor in self.motors:
                motor.update_quick_data_readout()
            time.sleep(0.01)
        
        self.stop_motors()
        print("Diagonal test completed")
    
    def run_all_tests(self):
        """Run all movement tests"""
        print("=== OMNIWHEEL MOVEMENT TESTS ===")
        print("Make sure the robot is on a safe surface and has room to move!")
        print("Press Ctrl+C at any time to stop tests")
        
        try:
            # Test individual motors first
            self.test_individual_motors(duration=2)
            time.sleep(1)
            
            # Test forward movement
            self.test_forward_movement(duration=3)
            time.sleep(1)
            
            # Test backward movement
            self.test_backward_movement(duration=3)
            time.sleep(1)
            
            # Test diagonal movement
            self.test_diagonal_movement(duration=3)
            
            print("\n=== ALL TESTS COMPLETED ===")
            print("Check that:")
            print("1. All motors rotated in the correct direction")
            print("2. Forward movement moved the robot forward")
            print("3. Backward movement moved the robot backward")
            print("4. Individual motors rotated when tested")
            print("5. Diagonal movement combined forward and strafe")
            
        except KeyboardInterrupt:
            print("\nTests interrupted by user")
        finally:
            self.stop_motors()
            print("All motors stopped")

def main():
    print("Omniwheel Test Program")
    print("This will test forward, backward, and diagonal movement")
    print("Make sure the robot is on a safe surface!")
    
    # Set force_calibration=True if you need to recalibrate motors
    force_calibration = False  # Change to True to force calibration
    
    if force_calibration:
        print("WARNING: Force calibration is enabled!")
        print("This will recalibrate all motors and may take several minutes.")
        input("Press Enter to continue with calibration, or Ctrl+C to cancel...")
    
    test = OmniwheelTest()
    
    if len(test.motors) < 4:
        print("ERROR: Could not initialize all motors. Exiting.")
        return
    
    # Ask user what to test
    print("\nWhat would you like to test?")
    print("1. All tests (recommended)")
    print("2. Forward movement only")
    print("3. Backward movement only")
    print("4. Individual motors only")
    print("5. Diagonal movement only")
    
    try:
        choice = input("Enter choice (1-5): ").strip()
        
        if choice == "1":
            test.run_all_tests()
        elif choice == "2":
            test.test_forward_movement(duration=5)
        elif choice == "3":
            test.test_backward_movement(duration=5)
        elif choice == "4":
            test.test_individual_motors(duration=3)
        elif choice == "5":
            test.test_diagonal_movement(duration=5)
        else:
            print("Invalid choice. Running all tests...")
            test.run_all_tests()
            
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    finally:
        test.stop_motors()
        print("Test program completed")

if __name__ == "__main__":
    main()
