#!/usr/bin/env python3
"""
RoboCup Soccer Robot - Integrated Control System
Combines ball detection with motor control for competitive play
"""

import sys
import time
from soccer_robot import SoccerRobot

def main():
    print("=== RoboCup Soccer Robot ===")
    print("Initializing robot systems...")
    
    try:
        # Create and run the robot
        robot = SoccerRobot()
        print("Robot initialized successfully!")
        print("Starting RoboCup match mode...")
        robot.run()
        
    except Exception as e:
        print(f"Error initializing robot: {e}")
        print("Check motor connections and camera setup")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 