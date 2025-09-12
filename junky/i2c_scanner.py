import board
import busio
import time

def scan_i2c_devices():
    """
    Scan for I2C devices and return a list of found addresses
    """
    print("Starting I2C scan...")
    
    # Initialize I2C bus
    i2c = busio.I2C(board.SCL, board.SDA)
    
    # Wait for I2C to be ready
    while not i2c.try_lock():
        pass
    
    found_devices = []
    
    # Scan all possible I2C addresses (0x08 to 0x77)
    for address in range(0x08, 0x78):
        try:
            # Try to read from the address
            result = i2c.writeto(address, bytes([0]))
            found_devices.append(address)
            print(f"Found device at address: 0x{address:02X} ({address})")
        except OSError:
            # No device at this address
            pass
    
    i2c.unlock()
    
    if not found_devices:
        print("No I2C devices found!")
    else:
        print(f"\nFound {len(found_devices)} I2C device(s):")
        for addr in found_devices:
            print(f"  - Address: 0x{addr:02X} (decimal: {addr})")
    
    return found_devices

def test_specific_address(address):
    """
    Test communication with a specific I2C address
    """
    print(f"\nTesting communication with address 0x{address:02X}...")
    
    i2c = busio.I2C(board.SCL, board.SDA)
    
    while not i2c.try_lock():
        pass
    
    try:
        # Try to read from the address
        result = i2c.writeto(address, bytes([0]))
        print(f"✓ Successfully communicated with device at 0x{address:02X}")
        
        # Try to read some data (this might fail for some devices)
        try:
            data = bytearray(1)
            i2c.readfrom_into(address, data)
            print(f"  Read data: {[hex(x) for x in data]}")
        except OSError:
            print("  Could not read data (this is normal for some devices)")
            
    except OSError as e:
        print(f"✗ Failed to communicate with device at 0x{address:02X}: {e}")
    
    i2c.unlock()

if __name__ == "__main__":
    print("=== I2C Device Scanner ===")
    print("This script will scan for all I2C devices on the bus.\n")
    
    # Scan for all devices
    devices = scan_i2c_devices()
    
    if devices:
        print("\n=== Testing Communication ===")
        for device in devices:
            test_specific_address(device)
    
    print("\n=== Scan Complete ===")
    print("Note: Some devices may not respond to all commands but still be functional.")
    print("Your motor drivers should be at addresses 10 and 11 based on your code.") 