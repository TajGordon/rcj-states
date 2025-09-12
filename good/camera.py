import multiprocessing as mp
from multiprocessing import Process, Queue, Event, Value, Array
import cv2
import numpy as np
import time
import math
from picamera2 import Picamera2
import threading

def camera_process(ball_position, ball_radius, ball_angle, ball_detected, stop_event, frame_queue=None, detection_mode='enhanced'):
    """
    Separate process that handles camera capture and ball detection.
    
    Args:
        ball_position: Shared array [x, y] for ball center position
        ball_radius: Shared value for ball radius
        ball_angle: Shared value for ball angle in radians (0 = straight forward, clockwise)
        ball_detected: Shared value (0/1) indicating if ball is detected
        stop_event: Event to signal process to stop
        frame_queue: Optional queue to send processed frames to other processes
        detection_mode: 'basic', 'enhanced', or 'adaptive' - different detection algorithms
    """
    try:
        # Initialize Pi Camera
        picam2 = Picamera2()
        picam2.configure(picam2.create_video_configuration(
            main={"size": (640, 640), "format": "RGB888"}
        ))
        picam2.start()
        
        # Enhanced ball detection parameters (orange ball) - multiple ranges for better detection
        orange_ranges = [
            (np.array([0, 132, 61]), np.array([14, 255, 255])),    # Original range
            (np.array([0, 100, 50]), np.array([20, 255, 255])),    # Wider range
            (np.array([0, 80, 40]), np.array([25, 255, 255])),     # Even wider
            (np.array([0, 60, 30]), np.array([30, 255, 255])),     # Very wide for poor lighting
        ]
        
        # Additional color ranges for different lighting conditions
        lighting_ranges = [
            (np.array([0, 50, 20]), np.array([35, 255, 255])),     # Low light
            (np.array([0, 40, 15]), np.array([40, 255, 255])),     # Very low light
        ]
        
        # Camera center for angle calculation
        camera_center_x = 320  # 640 / 2
        camera_center_y = 320  # 640 / 2
        
        # Detection parameters based on mode
        if detection_mode == 'basic':
            min_area, max_area = 100, 30000
            min_circularity = 0.3
        elif detection_mode == 'enhanced':
            min_area, max_area = 50, 50000
            min_circularity = 0.2
        else:  # adaptive
            min_area, max_area = 30, 80000
            min_circularity = 0.15
        
        print(f"Camera process started successfully - Detection mode: {detection_mode}")
        
        while not stop_event.is_set():
            try:
                # Capture frame from Pi Camera
                frame = picam2.capture_array()
                
                # ============================================================================
                # SPACE FOR FRIEND'S OBJECT DETECTION CODE
                # Add your object detection code here if needed
                # ============================================================================
                
                # Convert to HSV for ball detection
                hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                
                # Enhanced ball detection with multiple color ranges
                combined_mask = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)
                
                # Try primary orange ranges first
                for lower, upper in orange_ranges:
                    mask = cv2.inRange(hsv_frame, lower, upper)
                    combined_mask = cv2.bitwise_or(combined_mask, mask)
                
                # If no detection, try lighting compensation ranges
                if np.sum(combined_mask) < 100:  # Very few pixels detected
                    for lower, upper in lighting_ranges:
                        mask = cv2.inRange(hsv_frame, lower, upper)
                        combined_mask = cv2.bitwise_or(combined_mask, mask)
                
                # Apply morphological operations to clean up the mask
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
                combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
                
                # Additional noise reduction
                kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
                combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel_small)
                
                # Find contours
                contours, _ = cv2.findContours(combined_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                
                # Enhanced contour filtering with improved criteria
                filtered_contours = []
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if min_area <= area <= max_area:
                        # Check circularity (how round the shape is)
                        perimeter = cv2.arcLength(contour, True)
                        if perimeter > 0:
                            circularity = 4 * np.pi * area / (perimeter * perimeter)
                            if circularity >= min_circularity:
                                # Additional checks for better ball detection
                                x, y, w, h = cv2.boundingRect(contour)
                                aspect_ratio = float(w) / h
                                
                                # Check if contour is roughly circular
                                if 0.5 <= aspect_ratio <= 2.0:  # Not too elongated
                                    # Additional check: convexity (how "filled" the shape is)
                                    hull = cv2.convexHull(contour)
                                    hull_area = cv2.contourArea(hull)
                                    if hull_area > 0:
                                        convexity = area / hull_area
                                        if convexity >= 0.7:  # Shape should be mostly convex
                                            # Check solidity (area vs convex hull)
                                            solidity = area / cv2.contourArea(cv2.convexHull(contour))
                                            if solidity >= 0.6:  # Should be fairly solid
                                                filtered_contours.append(contour)
                
                if filtered_contours:
                    largest_contour = max(filtered_contours, key=cv2.contourArea)
                    (x, y), radius = cv2.minEnclosingCircle(largest_contour)
                    
                    # Calculate angle from camera center to ball
                    # Convert to normalized coordinates (-1 to 1)
                    norm_x = (x - camera_center_x) / camera_center_x
                    norm_y = (y - camera_center_y) / camera_center_y
                    
                    # Calculate angle in radians
                    # atan2 gives angle from positive x-axis, we want from positive y-axis (forward)
                    # and clockwise direction, so we adjust the calculation
                    angle = math.atan2(norm_x, -norm_y)  # -norm_y because y increases downward in image
                    
                    # Ensure angle is in range [0, 2π)
                    if angle < 0:
                        angle += 2 * math.pi
                    
                    # Update shared memory with ball position
                    with ball_position.get_lock():
                        ball_position[0] = int(x)
                        ball_position[1] = int(y)
                    
                    with ball_radius.get_lock():
                        ball_radius.value = int(radius)
                    
                    with ball_angle.get_lock():
                        ball_angle.value = angle
                    
                    with ball_detected.get_lock():
                        ball_detected.value = 1
                    
                    # Optional: send processed frame to queue for web server
                    if frame_queue is not None:
                        # Create display frame with ball detection overlay
                        display_frame = frame.copy()
                        center = (int(x), int(y))
                        cv2.circle(display_frame, center, int(radius), (0, 255, 0), 2)
                        cv2.circle(display_frame, center, 2, (0, 255, 0), -1)
                        cv2.putText(display_frame, f"Ball: ({center[0]}, {center[1]})", 
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        
                        # Try to put frame in queue (non-blocking)
                        try:
                            frame_queue.put_nowait(display_frame)
                        except:
                            pass  # Queue full, skip this frame
                else:
                    # No ball detected
                    with ball_detected.get_lock():
                        ball_detected.value = 0
                    
                    # Reset angle when no ball detected
                    with ball_angle.get_lock():
                        ball_angle.value = 0.0
                    
                    # Optional: send frame without ball
                    if frame_queue is not None:
                        display_frame = frame.copy()
                        cv2.putText(display_frame, "No ball detected", 
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                        try:
                            frame_queue.put_nowait(display_frame)
                        except:
                            pass
                
                time.sleep(0.033)  # ~30 FPS
                
            except Exception as e:
                print(f"Error in camera process: {e}")
                time.sleep(0.1)
                
    except Exception as e:
        print(f"Failed to initialize camera process: {e}")
    finally:
        try:
            picam2.stop()
            picam2.close()
        except:
            pass
        print("Camera process stopped")

class Camera:
    """
    Camera class that launches a separate process to read and process camera data.
    Provides interface to access ball position data from other processes.
    """
    
    def __init__(self, enable_frame_queue=False, detection_mode='enhanced'):
        """
        Initialize the camera system.
        
        Args:
            enable_frame_queue: If True, creates a queue for processed frames (useful for web server)
            detection_mode: 'basic', 'enhanced', or 'adaptive' - different ball detection algorithms
        """
        # Shared memory for ball position [x, y]
        self.ball_position = Array('i', [0, 0])
        
        # Shared memory for ball radius
        self.ball_radius = Value('i', 0)
        
        # Shared memory for ball angle in radians
        self.ball_angle = Value('d', 0.0)  # 'd' for double precision float
        
        # Shared memory for ball detection status (0 = not detected, 1 = detected)
        self.ball_detected = Value('i', 0)
        
        # Event to signal camera process to stop
        self.stop_event = Event()
        
        # Optional queue for processed frames
        self.frame_queue = Queue(maxsize=2) if enable_frame_queue else None
        
        # Camera process
        self.camera_process = None
        
        # Process state
        self.is_running = False
        
        # Detection mode
        self.detection_mode = detection_mode
        
    def start(self):
        """Start the camera process."""
        if self.is_running:
            print("Camera is already running")
            return
            
        try:
            self.camera_process = Process(
                target=camera_process,
                args=(self.ball_position, self.ball_radius, self.ball_angle, self.ball_detected, 
                      self.stop_event, self.frame_queue, self.detection_mode)
            )
            self.camera_process.start()
            self.is_running = True
            print("Camera process started")
            
            # Give the process a moment to initialize
            time.sleep(1)
            
        except Exception as e:
            print(f"Failed to start camera process: {e}")
            self.is_running = False
    
    def stop(self):
        """Stop the camera process."""
        if not self.is_running:
            print("Camera is not running")
            return
            
        try:
            self.stop_event.set()
            if self.camera_process:
                self.camera_process.join(timeout=5)
                if self.camera_process.is_alive():
                    self.camera_process.terminate()
                    self.camera_process.join()
            self.is_running = False
            print("Camera process stopped")
            
        except Exception as e:
            print(f"Error stopping camera process: {e}")

    def get_ball_position(self):
        """
        Get the current ball position on the camera screen.
        
        Returns:
            tuple: (x, y) coordinates of ball center, or (None, None) if no ball detected
        """
        if not self.is_running:
            return None, None
            
        with self.ball_detected.get_lock():
            detected = self.ball_detected.value
            
        if detected:
            with self.ball_position.get_lock():
                return self.ball_position[0], self.ball_position[1]
        else:
            return None, None
    
    def get_ball_radius(self):
        """
        Get the current ball radius in pixels.
        
        Returns:
            int: Ball radius in pixels, or 0 if no ball detected
        """
        if not self.is_running:
            return 0
            
        with self.ball_detected.get_lock():
            detected = self.ball_detected.value
            
        if detected:
            with self.ball_radius.get_lock():
                return self.ball_radius.value
        else:
            return 0
    
    def get_ball_angle(self):
        """
        Get the current ball angle in radians.
        
        Returns:
            float: Ball angle in radians (0 = straight forward, clockwise from 0 to 2π), 
                   or 0.0 if no ball detected
        """
        if not self.is_running:
            return 0.0
            
        with self.ball_detected.get_lock():
            detected = self.ball_detected.value
            
        if detected:
            with self.ball_angle.get_lock():
                return self.ball_angle.value
        else:
            return 0.0
    
    def is_ball_detected(self):
        """
        Check if a ball is currently detected.
        
        Returns:
            bool: True if ball is detected, False otherwise
        """
        if not self.is_running:
            return False
            
        with self.ball_detected.get_lock():
            return self.ball_detected.value == 1
    
    def get_processed_frame(self):
        """
        Get the latest processed frame from the camera process.
        Only available if enable_frame_queue=True was set during initialization.
        
        Returns:
            numpy.ndarray: Processed frame with ball detection overlay, or None if not available
        """
        if self.frame_queue is None:
            return None
            
        try:
            return self.frame_queue.get_nowait()
        except:
            return None
    
    def get_camera_info(self):
        """
        Get information about the camera system.
        
        Returns:
            dict: Dictionary containing camera status and ball information
        """
        ball_x, ball_y = self.get_ball_position()
        
        return {
            'is_running': self.is_running,
            'ball_detected': self.is_ball_detected(),
            'ball_position': (ball_x, ball_y),
            'ball_radius': self.get_ball_radius(),
            'ball_angle': self.get_ball_angle(),
            'frame_queue_available': self.frame_queue is not None,
            'detection_mode': self.detection_mode
        }
    
    
    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()

# Example usage and testing
def main():
    """Example usage of the Camera class."""
    print("Testing Camera class...")
    
    # Create camera instance with frame queue for web server
    camera = Camera(enable_frame_queue=True)
    
    try:
        # Start camera
        camera.start()
        
        # Run for 10 seconds, printing ball position every second
        for i in range(10):
            time.sleep(1)
            
            info = camera.get_camera_info()
            print(f"Camera Info: {info}")
            
            if info['ball_detected']:
                x, y = info['ball_position']
                radius = info['ball_radius']
                angle = info['ball_angle']
                angle_deg = math.degrees(angle)
                print(f"Ball detected at ({x}, {y}) with radius {radius}, angle {angle:.3f} rad ({angle_deg:.1f}°)")
            else:
                print("No ball detected")
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    finally:
        # Stop camera
        camera.stop()
        print("Camera test completed")

if __name__ == "__main__":
    main()