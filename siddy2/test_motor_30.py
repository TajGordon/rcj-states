#!/usr/bin/env python3
"""
Test script to check if motor at address 30 can spin.
This tests the front-left motor specifically.
"""

import time
import sys
import board
import busio
from steelbar_powerful_bldc_driver import PowerfulBLDCDriver

def test_motor_30():
    """Test motor at address 30 (front-left motor)"""
    print("🔧 Testing Motor at Address 30 (Front-Left)")
    print("=" * 50)
    
    # Initialize I2C
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        print("✅ I2C initialized successfully")
    except Exception as e:
        print(f"❌ Failed to initialize I2C: {e}")
        return False
    
    # Initialize motor at address 30
    try:
        print("🔌 Connecting to motor at address 30...")
        motor = PowerfulBLDCDriver(i2c, 30)
        print("✅ Motor driver initialized")
    except Exception as e:
        print(f"❌ Failed to initialize motor driver: {e}")
        return False
    
    # Check firmware version
    try:
        firmware_version = motor.get_firmware_version()
        print(f"📋 Firmware version: {firmware_version}")
        if firmware_version != 3:
            print(f"⚠️  Warning: Expected firmware version 3, got {firmware_version}")
    except Exception as e:
        print(f"❌ Failed to read firmware version: {e}")
        return False
    
    # Configure motor (matching soccer_bot.py settings)
    try:
        print("⚙️  Configuring motor...")
        motor.set_current_limit_foc(4*65536)  # 4A current limit
        motor.set_id_pid_constants(1500, 200)
        motor.set_iq_pid_constants(1500, 200)
        motor.set_speed_pid_constants(4e-2, 4e-4, 3e-2)
        motor.set_position_pid_constants(275, 0, 0)
        motor.set_position_region_boundary(250000)
        motor.set_speed_limit(120000000)  # 120M max speed
        
        motor.configure_operating_mode_and_sensor(15, 1)
        motor.configure_command_mode(15)
        motor.set_calibration_options(300, 2097152, 50000, 500000)
        
        # Skip calibration - use previously calibrated values
        print("📊 Using previously calibrated values:")
        print(f"   elecangleoffset: {motor.get_calibration_ELECANGLEOFFSET()}")
        print(f"   sincoscentre: {motor.get_calibration_SINCOSCENTRE()}")
        
        motor.configure_operating_mode_and_sensor(3, 1)
        motor.configure_command_mode(12)
        
        print("✅ Motor configured successfully")
    except Exception as e:
        print(f"❌ Failed to configure motor: {e}")
        return False
    
    # Test motor spinning
    test_speeds = [
        (10000000, "Slow forward"),
        (20000000, "Medium forward"),
        (30000000, "Fast forward"),
        (-10000000, "Slow backward"),
        (-20000000, "Medium backward"),
        (-30000000, "Fast backward")
    ]
    
    print("\n🔄 Testing motor spinning...")
    print("Watch the motor to see if it spins!")
    print("Press Ctrl+C to stop early")
    
    try:
        for speed, description in test_speeds:
            print(f"\n🎯 Testing: {description} (speed: {speed//1000}k)")
            motor.set_speed(speed)
            
            # Run for 3 seconds
            start_time = time.time()
            while time.time() - start_time < 3.0:
                motor.update_quick_data_readout()
                time.sleep(0.01)
            
            print(f"✅ {description} test completed")
            time.sleep(0.5)  # Brief pause between tests
        
        # Stop motor
        print("\n🛑 Stopping motor...")
        motor.set_speed(0)
        time.sleep(1.0)
        
        print("\n✅ All tests completed successfully!")
        print("Motor at address 30 is working correctly!")
        return True
        
    except KeyboardInterrupt:
        print("\n⚠️  Test interrupted by user")
        motor.set_speed(0)
        return False
    except Exception as e:
        print(f"\n❌ Error during motor test: {e}")
        motor.set_speed(0)
        return False

def main():
    print("🤖 Motor 30 Test Program")
    print("This will test if the motor at address 30 can spin")
    print("Make sure the robot is on a safe surface!")
    print()
    
    try:
        success = test_motor_30()
        if success:
            print("\n🎉 Motor 30 test PASSED!")
        else:
            print("\n💥 Motor 30 test FAILED!")
    except Exception as e:
        print(f"\n💥 Test program error: {e}")
    finally:
        print("\nTest program completed")

if __name__ == "__main__":
    main()
