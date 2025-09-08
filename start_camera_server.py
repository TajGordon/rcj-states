#!/usr/bin/env python3
"""
Simple startup script for the camera web server.
This script provides an easy way to start the camera web server with proper error handling.
"""

import sys
import os

def main():
    print("🤖 Robot Camera Web Server")
    print("=" * 40)
    
    try:
        # Import and run the camera web server
        from camera_web_server import CameraWebServer
        
        print("Initializing camera...")
        server = CameraWebServer()
        
        print("\n✅ Camera initialized successfully!")
        print("🌐 Starting web server...")
        print("\n📱 To view the camera feed:")
        print("   1. Open your web browser")
        print("   2. Navigate to: http://localhost:5000")
        print("   3. Or from another device: http://[YOUR_IP]:5000")
        print("\n⏹️  Press Ctrl+C to stop the server")
        print("-" * 40)
        
        # Run the server
        server.run(host='0.0.0.0', port=5000, debug=False)
        
    except ImportError as e:
        print(f"❌ Import Error: {e}")
        print("\n💡 Make sure you have installed the required dependencies:")
        print("   pip install -r requirements.txt")
        sys.exit(1)
        
    except Exception as e:
        print(f"❌ Error: {e}")
        print("\n💡 Common issues:")
        print("   - Make sure the camera is connected and not being used by another application")
        print("   - Check if you have the necessary permissions to access the camera")
        print("   - Ensure you're running this on a Raspberry Pi with Pi Camera")
        sys.exit(1)

if __name__ == "__main__":
    main()

