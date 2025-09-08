import board
import busio
import time
from steelbar_powerful_bldc_driver import PowerfulBLDCDriver

def check_motor_drivers():
    """
    Check for motor drivers at the addresses used in your code
    """
    print("=== Motor Driver I2C Check ===")
    
    # Initialize I2C bus
    i2c = busio.I2C(board.SCL, board.SDA)
    
    # Addresses from your code
    motor_addresses = [10, 11]  # From soccer_robot.py
    test_addresses = list(range(8, 120))  # From motor_test.py range check
    
    print(f"Checking motor driver addresses: {motor_addresses}")
    print(f"Also scanning range: {test_addresses[0]} to {test_addresses[-1]}")
    print()
    
    found_motors = []
    
    # Check specific motor addresses first
    for addr in motor_addresses:
        print(f"Testing motor address {addr} (0x{addr:02X})...")
        try:
            motor = PowerfulBLDCDriver(i2c, addr)
            version = motor.get_firmware_version()
            print(f"  ✓ Found motor driver at address {addr}")
            print(f"    Firmware version: {version}")
            if version == 3:
                print("    ✓ Compatible firmware version")
                found_motors.append(addr)
            else:
                print(f"    ⚠ Incompatible firmware version: {version}")
        except Exception as e:
            print(f"  ✗ No motor driver at address {addr}: {e}")
    
    print("\n=== Full I2C Scan ===")
    found_devices = []
    
    # Wait for I2C to be ready
    while not i2c.try_lock():
        pass
    
    # Scan all addresses in the test range
    for address in test_addresses:
        try:
            # Try to read from the address
            result = i2c.writeto(address, bytes([0]))
            found_devices.append(address)
            print(f"Found device at address: {address} (0x{address:02X})")
        except OSError:
            # No device at this address
            pass
    
    i2c.unlock()
    
    print(f"\n=== Summary ===")
    print(f"Motor drivers found: {found_motors}")
    print(f"Total I2C devices found: {len(found_devices)}")
    print(f"All device addresses: {found_devices}")
    
    if found_motors:
        print("\n✓ Motor drivers are properly connected!")
    else:
        print("\n⚠ No compatible motor drivers found!")
        print("Check your wiring and I2C connections.")
    
    return found_motors, found_devices

def test_motor_communication(addr):
    """
    Test basic communication with a motor driver
    """
    print(f"\n=== Testing Motor at Address {addr} ===")
    
    i2c = busio.I2C(board.SCL, board.SDA)
    
    try:
        motor = PowerfulBLDCDriver(i2c, addr)
        
        # Test basic communication
        version = motor.get_firmware_version()
        print(f"Firmware version: {version}")
        
        # Test some basic commands
        motor.set_current_limit_foc(65536)
        print("✓ Set current limit")
        
        motor.set_speed_limit(10000000)
        print("✓ Set speed limit")
        
        print("✓ Motor communication test successful!")
        return True
        
    except Exception as e:
        print(f"✗ Motor communication failed: {e}")
        return False

if __name__ == "__main__":
    found_motors, all_devices = check_motor_drivers()
    
    # Test communication with found motors
    if found_motors:
        print("\n=== Testing Motor Communication ===")
        for motor_addr in found_motors:
            test_motor_communication(motor_addr)
    
    print("\n=== Check Complete ===") 