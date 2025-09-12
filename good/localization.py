import config
import math
import time
from typing import List, Tuple, Optional

class Localization:
    def __init__(self):
        # Algorithm parameters
        self.step_size = 1.0  # mm - how far to step when searching
        self.max_iterations = 50
        self.convergence_threshold = 0.5  # mm - stop when improvement is less than this
        self.max_raycast_distance = 3000  # mm - maximum distance to raycast (diagonal of field)
        
        # Current best estimate
        self.best_position = (config.field_center_x, config.field_center_y)  # (x, y) in mm
        self.best_error = float('inf')
        
        # Performance tracking
        self.last_update_time = 0
        self.iteration_count = 0
        
        # Pre-compute field geometry for efficient raycasting
        self._setup_field_geometry()
        
    def estimate_position(self, imu_heading: float, tof_pairs: List[Tuple[float, float]]) -> Tuple[float, float, float]:
        """
        Estimate robot position using iterative raycasting algorithm.
        
        Args:
            imu_heading: Robot heading in radians (from IMU)
            tof_pairs: List of (angle_rad, distance_mm) from ToF sensors
            
        Returns:
            (x, y, heading) - estimated position and orientation
        """
        if not tof_pairs:
            return self.best_position[0], self.best_position[1], imu_heading
            
        # Use previous best position as starting point
        current_pos = self.best_position
        current_error = self._calculate_error(current_pos, imu_heading, tof_pairs)
        
        # Iterative improvement
        improved = True
        iteration = 0
        
        while improved and iteration < self.max_iterations:
            improved = False
            iteration += 1
            
            # Try positions around current position
            for dx in [-self.step_size, 0, self.step_size]:
                for dy in [-self.step_size, 0, self.step_size]:
                    if dx == 0 and dy == 0:
                        continue
                        
                    test_pos = (current_pos[0] + dx, current_pos[1] + dy)
                    
                    # Check if position is within field bounds
                    if not self._is_valid_position(test_pos):
                        continue
                    
                    # Calculate error for this position
                    test_error = self._calculate_error(test_pos, imu_heading, tof_pairs)
                    
                    # If this position is better, use it
                    if test_error < current_error:
                        current_pos = test_pos
                        current_error = test_error
                        improved = True
            
            # Check for convergence
            if not improved or current_error < self.convergence_threshold:
                break
        
        # Update best estimate
        if current_error < self.best_error:
            self.best_position = current_pos
            self.best_error = current_error
        
        self.iteration_count = iteration
        self.last_update_time = time.time()
        
        return current_pos[0], current_pos[1], imu_heading
    
    def _calculate_error(self, position: Tuple[float, float], heading: float, tof_pairs: List[Tuple[float, float]]) -> float:
        """
        Calculate error between actual ToF readings and theoretical readings at given position.
        
        Args:
            position: (x, y) position to test
            heading: Robot heading in radians
            tof_pairs: List of (angle_rad, distance_mm) from ToF sensors
            
        Returns:
            Total error (lower is better)
        """
        total_error = 0.0
        valid_readings = 0
        
        for sensor_angle, measured_distance in tof_pairs:
            # Calculate theoretical distance at this position
            theoretical_distance = self._raycast_distance(position, heading + sensor_angle)
            
            if theoretical_distance is not None and measured_distance > 0:
                # Calculate error (squared difference for better convergence)
                error = (measured_distance - theoretical_distance) ** 2
                total_error += error
                valid_readings += 1
        
        # Return average error, or high error if no valid readings
        if valid_readings == 0:
            return float('inf')
        
        return total_error / valid_readings
    
    def _setup_field_geometry(self):
        """Pre-compute field geometry for efficient raycasting."""
        # Field boundaries (250mm from walls)
        self.field_bounds = {
            'left': config.field_boundary_left,
            'right': config.field_boundary_right,
            'top': config.field_boundary_top,
            'bottom': config.field_boundary_bottom
        }
        
        # Goal geometry (left and right goals)
        self.goals = [
            # Left goal
            {
                'x_min': config.goal_left_x,
                'x_max': config.goal_left_x + config.goal_depth,
                'y_min': config.goal_center_y - config.goal_width / 2,
                'y_max': config.goal_center_y + config.goal_width / 2
            },
            # Right goal
            {
                'x_min': config.goal_right_x - config.goal_depth,
                'x_max': config.goal_right_x,
                'y_min': config.goal_center_y - config.goal_width / 2,
                'y_max': config.goal_center_y + config.goal_width / 2
            }
        ]
        
        # Goal crossbars (140mm above playing surface, 20mm deep)
        self.goal_crossbars = [
            # Left goal crossbar
            {
                'x_min': config.goal_left_x,
                'x_max': config.goal_left_x + config.goal_crossbar_depth,
                'y_min': config.goal_center_y - config.goal_width / 2,
                'y_max': config.goal_center_y + config.goal_width / 2,
                'height': config.goal_crossbar_height  # Height above playing surface
            },
            # Right goal crossbar
            {
                'x_min': config.goal_right_x - config.goal_crossbar_depth,
                'x_max': config.goal_right_x,
                'y_min': config.goal_center_y - config.goal_width / 2,
                'y_max': config.goal_center_y + config.goal_width / 2,
                'height': config.goal_crossbar_height  # Height above playing surface
            }
        ]
        
        # Goal exclusion zones
        self.goal_exclusions = [
            # Left goal exclusion
            {
                'x_min': config.goal_left_x + config.goal_depth,
                'x_max': config.goal_left_exclusion_x,
                'y_min': config.goal_left_exclusion_y_min,
                'y_max': config.goal_left_exclusion_y_max
            },
            # Right goal exclusion
            {
                'x_min': config.goal_right_exclusion_x,
                'x_max': config.goal_right_x - config.goal_depth,
                'y_min': config.goal_right_exclusion_y_min,
                'y_max': config.goal_right_exclusion_y_max
            }
        ]

    def _raycast_distance(self, position: Tuple[float, float], ray_angle: float) -> Optional[float]:
        """
        Efficient raycast from position in given direction to find distance to nearest obstacle.
        Uses analytical intersection with field boundaries and goals.
        
        Args:
            position: (x, y) starting position
            ray_angle: Direction to raycast (radians, 0 = north, counter-clockwise)
            
        Returns:
            Distance to nearest obstacle in mm, or None if no obstacle found
        """
        x, y = position
        
        # Calculate ray direction
        dx = math.sin(ray_angle)  # sin because 0 = north, counter-clockwise
        dy = -math.cos(ray_angle)  # negative cos because y increases downward
        
        # Avoid division by zero
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return None
        
        min_distance = float('inf')
        
        # Check field boundaries
        if dx != 0:
            # Left boundary
            t = (self.field_bounds['left'] - x) / dx
            if t > 0:
                intersection_y = y + dy * t
                if self.field_bounds['top'] <= intersection_y <= self.field_bounds['bottom']:
                    min_distance = min(min_distance, t)
            
            # Right boundary
            t = (self.field_bounds['right'] - x) / dx
            if t > 0:
                intersection_y = y + dy * t
                if self.field_bounds['top'] <= intersection_y <= self.field_bounds['bottom']:
                    min_distance = min(min_distance, t)
        
        if dy != 0:
            # Top boundary
            t = (self.field_bounds['top'] - y) / dy
            if t > 0:
                intersection_x = x + dx * t
                if self.field_bounds['left'] <= intersection_x <= self.field_bounds['right']:
                    min_distance = min(min_distance, t)
            
            # Bottom boundary
            t = (self.field_bounds['bottom'] - y) / dy
            if t > 0:
                intersection_x = x + dx * t
                if self.field_bounds['left'] <= intersection_x <= self.field_bounds['right']:
                    min_distance = min(min_distance, t)
        
        # Check goal intersections
        for goal in self.goals:
            # Check horizontal goal walls
            if dy != 0:
                # Top wall
                t = (goal['y_min'] - y) / dy
                if t > 0:
                    intersection_x = x + dx * t
                    if goal['x_min'] <= intersection_x <= goal['x_max']:
                        min_distance = min(min_distance, t)
                
                # Bottom wall
                t = (goal['y_max'] - y) / dy
                if t > 0:
                    intersection_x = x + dx * t
                    if goal['x_min'] <= intersection_x <= goal['x_max']:
                        min_distance = min(min_distance, t)
            
            # Check vertical goal walls
            if dx != 0:
                # Left wall
                t = (goal['x_min'] - x) / dx
                if t > 0:
                    intersection_y = y + dy * t
                    if goal['y_min'] <= intersection_y <= goal['y_max']:
                        min_distance = min(min_distance, t)
                
                # Right wall
                t = (goal['x_max'] - x) / dx
                if t > 0:
                    intersection_y = y + dy * t
                    if goal['y_min'] <= intersection_y <= goal['y_max']:
                        min_distance = min(min_distance, t)
        
        # Check goal crossbars (140mm above playing surface)
        # Note: This assumes ToF sensors are mounted at robot height (~50-100mm)
        # and can detect the crossbar at 140mm height
        for crossbar in self.goal_crossbars:
            # For simplicity, we'll treat crossbars as 2D obstacles at ground level
            # In reality, you might need 3D raycasting for accurate crossbar detection
            # Check horizontal crossbar edges
            if dy != 0:
                # Top edge
                t = (crossbar['y_min'] - y) / dy
                if t > 0:
                    intersection_x = x + dx * t
                    if crossbar['x_min'] <= intersection_x <= crossbar['x_max']:
                        min_distance = min(min_distance, t)
                
                # Bottom edge
                t = (crossbar['y_max'] - y) / dy
                if t > 0:
                    intersection_x = x + dx * t
                    if crossbar['x_min'] <= intersection_x <= crossbar['x_max']:
                        min_distance = min(min_distance, t)
            
            # Check vertical crossbar edges
            if dx != 0:
                # Left edge
                t = (crossbar['x_min'] - x) / dx
                if t > 0:
                    intersection_y = y + dy * t
                    if crossbar['y_min'] <= intersection_y <= crossbar['y_max']:
                        min_distance = min(min_distance, t)
                
                # Right edge
                t = (crossbar['x_max'] - x) / dx
                if t > 0:
                    intersection_y = y + dy * t
                    if crossbar['y_min'] <= intersection_y <= crossbar['y_max']:
                        min_distance = min(min_distance, t)
        
        # Check goal exclusion zones
        for exclusion in self.goal_exclusions:
            # Check horizontal exclusion walls
            if dy != 0:
                # Top wall
                t = (exclusion['y_min'] - y) / dy
                if t > 0:
                    intersection_x = x + dx * t
                    if exclusion['x_min'] <= intersection_x <= exclusion['x_max']:
                        min_distance = min(min_distance, t)
                
                # Bottom wall
                t = (exclusion['y_max'] - y) / dy
                if t > 0:
                    intersection_x = x + dx * t
                    if exclusion['x_min'] <= intersection_x <= exclusion['x_max']:
                        min_distance = min(min_distance, t)
            
            # Check vertical exclusion walls
            if dx != 0:
                # Left wall
                t = (exclusion['x_min'] - x) / dx
                if t > 0:
                    intersection_y = y + dy * t
                    if exclusion['y_min'] <= intersection_y <= exclusion['y_max']:
                        min_distance = min(min_distance, t)
                
                # Right wall
                t = (exclusion['x_max'] - x) / dx
                if t > 0:
                    intersection_y = y + dy * t
                    if exclusion['y_min'] <= intersection_y <= exclusion['y_max']:
                        min_distance = min(min_distance, t)
        
        if min_distance < float('inf') and min_distance <= self.max_raycast_distance:
            return min_distance
        
        return None  # No obstacle found within max distance
    
    def _is_obstacle_at(self, x: float, y: float) -> bool:
        """
        Check if there's an obstacle at the given position.
        
        Args:
            x, y: Position to check
            
        Returns:
            True if there's an obstacle at this position
        """
        # Check field boundaries (250mm from walls)
        if (x < self.field_bounds['left'] or x > self.field_bounds['right'] or 
            y < self.field_bounds['top'] or y > self.field_bounds['bottom']):
            return True
        
        # Check goal areas
        for goal in self.goals:
            if (goal['x_min'] <= x <= goal['x_max'] and 
                goal['y_min'] <= y <= goal['y_max']):
                return True
        
        # Check goal exclusion zones
        for exclusion in self.goal_exclusions:
            if (exclusion['x_min'] <= x <= exclusion['x_max'] and 
                exclusion['y_min'] <= y <= exclusion['y_max']):
                return True
        
        return False
    
    def _is_valid_position(self, position: Tuple[float, float]) -> bool:
        """
        Check if a position is valid (within playable area).
        
        Args:
            position: (x, y) position to check
            
        Returns:
            True if position is valid
        """
        x, y = position
        
        # Must be within field boundaries (250mm from walls)
        if (x < self.field_bounds['left'] or x > self.field_bounds['right'] or 
            y < self.field_bounds['top'] or y > self.field_bounds['bottom']):
            return False
        
        # Must not be in goal areas
        for goal in self.goals:
            if (goal['x_min'] <= x <= goal['x_max'] and 
                goal['y_min'] <= y <= goal['y_max']):
                return False
        
        # Must not be in goal exclusion zones
        for exclusion in self.goal_exclusions:
            if (exclusion['x_min'] <= x <= exclusion['x_max'] and 
                exclusion['y_min'] <= y <= exclusion['y_max']):
                return False
        
        return True
    
    def get_status(self) -> dict:
        """Get localization status information."""
        return {
            'best_position': self.best_position,
            'best_error': self.best_error,
            'last_iterations': self.iteration_count,
            'last_update_time': self.last_update_time
        }
    
    def reset(self):
        """Reset localization to center of field."""
        self.best_position = (config.field_center_x, config.field_center_y)
        self.best_error = float('inf')
        self.iteration_count = 0

    