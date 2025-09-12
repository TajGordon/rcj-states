#!/usr/bin/env python3
"""
Startup script for the Real IMU Web Server.
This server provides a web interface to view IMU data with corrected clockwise angle handling.
"""

import sys
import os

# Add current directory to path so we can import our modules
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from real_imu_web_server import main
    print("Starting Real IMU Web Server...")
    print("This server provides corrected counter-clockwise angle handling for the BNO085 IMU.")
    print("Open http://localhost:5051 in your browser to view the IMU data.")
    print("Press Ctrl+C to stop the server.")
    main()
except ImportError as e:
    print(f"Error importing required modules: {e}")
    print("Make sure you have installed the required dependencies:")
    print("pip install flask flask-sock adafruit-circuitpython-bno08x")
    sys.exit(1)
except KeyboardInterrupt:
    print("\nServer stopped by user.")
    sys.exit(0)
except Exception as e:
    print(f"Error starting server: {e}")
    sys.exit(1)
