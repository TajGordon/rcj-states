#!/usr/bin/env python3
"""
Simple Omniwheel Motor Test - Custom Configuration

IMPORTANT: In this robot's configuration:
- Left/Right motors control FORWARD/BACKWARD movement
- Front/Back motors control STRAFING (left/right) movement
- Front motor (30) is INVERTED to compensate for hardware wiring
- Left motor (26) is INVERTED to compensate for hardware wiring
"""

import time
import sys
from steelbar_powerful_bldc_driver import PowerfulBLDCDriver
import board
import busio

class SimpleMotorTest:
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.motors = []
        self.motor_addresses = [27, 28, 30, 26]  # back, right, front, left
        self.test_speed = 2000000  # Conservative test speed

    def setup_motors(self):
        """Initialize all 4 motors"""
        print("Setting up motors...")
        print("Motor addresses: [27, 28, 30, 26] (back, right, front, left)")

        for i, addr in enumerate(self.motor_addresses):
            try:
                motor = PowerfulBLDCDriver(self.i2c, addr)

                if motor.get_firmware_version() != 3:
                    print(f"✗ Motor {i} (addr {addr}): Wrong firmware")
                    return False

                # Basic setup
                motor.set_current_limit_foc(65536)
                motor.set_speed_limit(8000000)
                motor.configure_operating_mode_and_sensor(3, 1)
                motor.configure_command_mode(12)

                self.motors.append(motor)
                print(f"✓ Motor {i} (addr {addr}): OK")

            except Exception as e:
                print(f"✗ Motor {i} (addr {addr}): Failed - {e}")
                return False

        print(f"✓ All {len(self.motors)} motors ready!")
        return len(self.motors) == 4

    def stop_motors(self):
        """Stop all motors"""
        for motor in self.motors:
            motor.set_speed(0)

    def test_movement(self, name, speeds, duration=2):
        """Test a specific movement pattern"""
        print(f"\n🧪 Testing {name}...")
        print(f"Speeds: {speeds} [back, right, front, left]")

        # Set motor speeds
        for i, speed in enumerate(speeds):
            if i < len(self.motors):
                self.motors[i].set_speed(speed)

        # Run for specified duration
        time.sleep(duration)

        # Stop motors
        self.stop_motors()
        print(f"✓ {name} test complete")

        # Brief pause
        time.sleep(0.5)

    def run_all_tests(self):
        """Run all movement tests"""

        if not self.setup_motors():
            print("❌ Motor setup failed!")
            return

        print("\n" + "="*60)
        print("🚀 CUSTOM OMNIWHEEL CONFIGURATION TESTS")
        print("="*60)
        print("⚠️  IMPORTANT: Left/Right motors = Forward/Backward")
        print("⚠️  Front/Back motors = Strafing Left/Right")
        print("⚠️  Front motor (30) is INVERTED to compensate for hardware wiring")
        print("⚠️  Left motor (26) is INVERTED to compensate for hardware wiring")
        print("Make sure robot is in open space!")
        print("Press Ctrl+C to stop at any time")
        print()

        try:
            # Test 1: Forward
            # Left and right motors forward, front/back stopped
            # Left motor inverted for correct direction
            self.test_movement("FORWARD", [0, self.test_speed, 0, -self.test_speed])

            # Test 2: Backward
            # Left and right motors backward, front/back stopped
            # Left motor inverted for correct direction
            self.test_movement("BACKWARD", [0, -self.test_speed, 0, self.test_speed])

            # Test 3: Strafe Right
            # Front and back motors rightward, left/right stopped
            # Front motor inverted for correct direction
            self.test_movement("STRAFE RIGHT", [self.test_speed, 0, -self.test_speed, 0])

            # Test 4: Strafe Left
            # Front and back motors leftward, left/right stopped
            # Front motor inverted for correct direction
            self.test_movement("STRAFE LEFT", [-self.test_speed, 0, self.test_speed, 0])

            # Test 5: Spin Clockwise
            # For pure spinning: opposite motors move in opposite directions
            # [back, right, front, left] = [left, backward, right, forward]
            self.test_movement("SPIN CLOCKWISE", [-self.test_speed, -self.test_speed, -self.test_speed, -self.test_speed])

            # Test 6: Spin Counter-clockwise
            # Opposite of clockwise: [right, forward, left, backward]
            self.test_movement("SPIN COUNTER-CLOCKWISE", [self.test_speed, self.test_speed, self.test_speed, self.test_speed])

            print("\n" + "="*60)
            print("✅ ALL TESTS COMPLETE!")
            print("="*60)

        except KeyboardInterrupt:
            print("\n⏹️  Test interrupted by user")
        finally:
            self.stop_motors()
            print("🛑 All motors stopped")

def main():
    print("🔧 Custom Omniwheel Motor Test")
    print("⚠️  Left/Right motors = Forward/Backward movement")
    print("⚠️  Front/Back motors = Strafing movement")
    print()

    tester = SimpleMotorTest()

    input("Press Enter to start tests... (Ctrl+C to stop)")

    tester.run_all_tests()

if __name__ == "__main__":
    main()
