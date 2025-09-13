#!/usr/bin/env python3
"""
Simple script to run the camera viewer for soccer_bot.py
"""

from camera_viewer import CameraViewer

if __name__ == "__main__":
    print("Starting Soccer Bot Camera Viewer...")
    print("This will show the camera feed with ball detection overlay.")
    print("Press 'q' to quit, 's' to save current frame.")
    print("=" * 50)
    
    viewer = CameraViewer()
    viewer.run()
