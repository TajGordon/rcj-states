#!/usr/bin/env python3

import sys
import time
from soccer_robot import SoccerRobot

def main():
    print("=== robocup soccer robot ===")
    print("initializing robot systems...")
    
    try:
        robot = SoccerRobot()
        print("robot initialized successfully!")
        print("starting robocup match mode...")
        robot.run()
        
    except Exception as e:
        print(f"error initializing robot: {e}")
        print("check motor connections and camera setup")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 