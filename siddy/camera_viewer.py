#!/usr/bin/env python3
"""
Camera viewer for soccer_bot.py to visualize ball detection in real-time.
Shows the camera feed with ball detection overlay and HSV mask.
"""

import cv2
import numpy as np
import time
from picamera2 import Picamera2

class CameraViewer:
    def __init__(self):
        # Initialize camera
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_video_configuration(
            main={"size": (640, 480), "format": "RGB888"}
        ))
        self.picam2.start()
        
        # HSV range for orange ball detection (same as soccer_bot.py)
        self.lower_orange = np.array([0, 50, 30])
        self.upper_orange = np.array([25, 255, 255])
        
        # Frame center
        self.frame_center_x = 320
        self.frame_center_y = 240
        
        print("Camera viewer started. Press 'q' to quit, 's' to save image.")
        print("Window shows: Original | HSV Mask | Detection Overlay")
    
    def detect_ball(self, frame):
        """Detect ball in frame and return detection info."""
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Count orange pixels
        orange_pixels = np.sum(mask > 0)
        
        # Filter contours by area
        filtered_contours = [x for x in contours if cv2.contourArea(x) > 5 and cv2.contourArea(x) < 50000]
        
        ball_detected = False
        ball_center = None
        ball_radius = 0
        
        if filtered_contours:
            largest_contour = max(filtered_contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            ball_center = (int(x), int(y))
            ball_radius = int(radius)
            ball_detected = True
        
        return ball_detected, ball_center, ball_radius, mask, orange_pixels, len(contours), len(filtered_contours)
    
    def create_overlay_frame(self, frame, ball_detected, ball_center, ball_radius, mask):
        """Create frame with ball detection overlay."""
        overlay_frame = frame.copy()
        
        if ball_detected:
            # Draw ball detection
            cv2.circle(overlay_frame, ball_center, ball_radius, (0, 255, 0), 2)
            cv2.circle(overlay_frame, ball_center, 2, (0, 255, 0), -1)
            
            # Draw center crosshair
            cv2.line(overlay_frame, (self.frame_center_x - 10, self.frame_center_y), 
                    (self.frame_center_x + 10, self.frame_center_y), (255, 0, 0), 1)
            cv2.line(overlay_frame, (self.frame_center_x, self.frame_center_y - 10), 
                    (self.frame_center_x, self.frame_center_y + 10), (255, 0, 0), 1)
            
            # Draw line from center to ball
            cv2.line(overlay_frame, (self.frame_center_x, self.frame_center_y), 
                    ball_center, (255, 255, 0), 2)
            
            # Add text info
            cv2.putText(overlay_frame, f"Ball: ({ball_center[0]}, {ball_center[1]})", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(overlay_frame, f"Radius: {ball_radius}", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            # No ball detected
            cv2.putText(overlay_frame, "No ball detected", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        return overlay_frame
    
    def run(self):
        """Main camera viewer loop."""
        try:
            while True:
                # Capture frame
                frame = self.picam2.capture_array()
                
                # Detect ball
                ball_detected, ball_center, ball_radius, mask, orange_pixels, total_contours, filtered_contours = self.detect_ball(frame)
                
                # Create overlay frame
                overlay_frame = self.create_overlay_frame(frame, ball_detected, ball_center, ball_radius, mask)
                
                # Convert mask to 3-channel for display
                mask_display = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
                
                # Add detection info to mask
                cv2.putText(mask_display, f"Orange pixels: {orange_pixels}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(mask_display, f"Contours: {total_contours} -> {filtered_contours}", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Resize frames for display
                frame_small = cv2.resize(frame, (320, 240))
                mask_small = cv2.resize(mask_display, (320, 240))
                overlay_small = cv2.resize(overlay_frame, (320, 240))
                
                # Combine frames horizontally
                combined = np.hstack([frame_small, mask_small, overlay_small])
                
                # Add titles
                cv2.putText(combined, "Original", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(combined, "HSV Mask", (330, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(combined, "Detection", (650, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Display combined frame
                cv2.imshow('Soccer Bot Camera Viewer', combined)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    # Save current frame
                    timestamp = int(time.time())
                    cv2.imwrite(f'soccer_bot_frame_{timestamp}.jpg', frame)
                    cv2.imwrite(f'soccer_bot_mask_{timestamp}.jpg', mask)
                    cv2.imwrite(f'soccer_bot_overlay_{timestamp}.jpg', overlay_frame)
                    print(f"Saved frames with timestamp {timestamp}")
                
                # Print detection info to console
                if ball_detected:
                    error_x = ball_center[0] - self.frame_center_x
                    error_y = self.frame_center_y - ball_center[1]
                    print(f"Ball detected: center=({ball_center[0]}, {ball_center[1]}), radius={ball_radius}, error=({error_x}, {error_y})")
                else:
                    print(f"No ball: orange_pixels={orange_pixels}, contours={total_contours}->{filtered_contours}")
                
                time.sleep(0.033)  # ~30 FPS
                
        except KeyboardInterrupt:
            print("\nStopping camera viewer...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources."""
        cv2.destroyAllWindows()
        self.picam2.stop()
        print("Camera viewer stopped.")

if __name__ == "__main__":
    viewer = CameraViewer()
    viewer.run()
