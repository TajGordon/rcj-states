import config
import math
import time

# all named agent so u just change from where you import
class Agent:
    def __init__(self, bot):
        """
        Initialize the blind ball chaser agent.
        
        Args:
            bot: Bot instance with access to camera, motor_controller, imu, tof, localization
        """
        self.bot = bot
        self.running = False
        
        # Movement parameters - REDUCED SPEEDS for smoother movement
        self.base_speed_level = 2.0  # Base speed when chasing ball (reduced from 6)
        self.slow_speed_level = 1.0  # Speed when close to ball (reduced from 3)
        self.fast_speed_level = 3.0  # Speed when far from ball (reduced from 8)
        
        # Distance thresholds (in pixels)
        self.close_distance = 80   # Close to ball
        self.far_distance = 200    # Far from ball
        
        # Boundary safety margins (in mm)
        self.boundary_margin = 100  # Stay 100mm away from field boundaries
        
        # Movement state management
        self.last_ball_angle = 0.0
        self.last_ball_distance = 0.0
        self.no_ball_count = 0
        self.max_no_ball_count = 10  # Stop after 10 cycles without ball
        
        # Movement state tracking to reduce command frequency
        self.current_movement_state = 'stopped'  # 'stopped', 'moving', 'turning'
        self.last_command_time = 0
        self.min_command_interval = 0.1  # Minimum 100ms between commands
        self.last_command_direction = 0.0
        self.last_command_speed = 0.0
        
        # Movement thresholds to avoid micro-adjustments
        self.min_angle_change = math.radians(5)  # 5 degrees minimum change
        self.min_speed_change = 0.5  # Minimum speed level change
        self.min_distance_change = 20  # Minimum distance change in pixels
        
        # Error handling
        self.consecutive_errors = 0
        self.max_consecutive_errors = 5
        
    def run(self, target_goal):
        """
        Main agent loop - blindly chase the ball while avoiding boundaries.
        
        Args:
            target_goal: 'blue' or 'yellow' - which goal to defend (not used in blind chaser)
        """
        self.running = True
        print(f"Blind Ball Chaser started - Target goal: {target_goal}")
        print(f"Movement parameters: base={self.base_speed_level}, slow={self.slow_speed_level}, fast={self.fast_speed_level}")
        
        try:
            while self.running:
                self._chase_ball_cycle()
                time.sleep(0.1)  # Reduced to 10 Hz main loop for smoother operation
                
        except KeyboardInterrupt:
            print("Blind Ball Chaser interrupted")
        except Exception as e:
            print(f"Error in agent loop: {e}")
            self.consecutive_errors += 1
            if self.consecutive_errors >= self.max_consecutive_errors:
                print("Too many consecutive errors, stopping agent")
                self.running = False
        finally:
            self.stop()
    
    def _chase_ball_cycle(self):
        """Single cycle of ball chasing logic with movement state management."""
        try:
            # Get current robot position and heading
            robot_x, robot_y, robot_heading = self._get_robot_state()
            
            # Get ball information
            ball_detected = self.bot.camera.is_ball_detected()
            
            if ball_detected:
                self.no_ball_count = 0
                self.consecutive_errors = 0  # Reset error count on successful detection
                
                # Get ball data
                ball_angle = self.bot.camera.get_ball_angle()
                ball_distance = self.bot.camera.get_ball_distance_from_center()
                
                # Check if movement is needed (avoid micro-adjustments)
                if self._should_move(ball_angle, ball_distance):
                    # Check if we're too close to boundaries
                    if self._is_near_boundary(robot_x, robot_y):
                        self._avoid_boundary(robot_x, robot_y, robot_heading)
                    else:
                        # Chase the ball
                        self._chase_ball(ball_angle, ball_distance)
                
                # Store for reference
                self.last_ball_angle = ball_angle
                self.last_ball_distance = ball_distance
            else:
                # No ball detected
                self.no_ball_count += 1
                
                if self.no_ball_count > self.max_no_ball_count:
                    # Stop if no ball for too long
                    self._safe_motor_command('stop')
                    print("No ball detected for too long - stopping")
                else:
                    # Use last known ball position or search
                    if self.last_ball_angle != 0.0:
                        print(f"No ball - using last known angle: {math.degrees(self.last_ball_angle):.1f}°")
                        self._chase_ball(self.last_ball_angle, self.last_ball_distance)
                    else:
                        # Search by turning
                        self._search_for_ball()
                        
        except Exception as e:
            print(f"Error in chase ball cycle: {e}")
            self.consecutive_errors += 1
            if self.consecutive_errors >= self.max_consecutive_errors:
                print("Too many consecutive errors, stopping")
                self.running = False
    
    def _should_move(self, ball_angle, ball_distance):
        """Check if movement is needed based on thresholds to avoid micro-adjustments."""
        current_time = time.time()
        
        # Check minimum time interval
        if current_time - self.last_command_time < self.min_command_interval:
            return False
        
        # Check if angle change is significant enough
        angle_change = abs(ball_angle - self.last_command_direction)
        if angle_change < self.min_angle_change:
            return False
        
        # Check if distance change is significant enough
        distance_change = abs(ball_distance - self.last_ball_distance)
        if distance_change < self.min_distance_change:
            return False
        
        return True
    
    def _safe_motor_command(self, command_type, *args):
        """Safely execute motor commands with error handling."""
        try:
            current_time = time.time()
            
            if command_type == 'stop':
                self.bot.motor_controller.stop_motors()
                self.current_movement_state = 'stopped'
                self.last_command_time = current_time
                print("Motors stopped")
                
            elif command_type == 'move_direction':
                direction, speed = args
                self.bot.motor_controller.move_direction(direction, speed)
                self.current_movement_state = 'moving'
                self.last_command_time = current_time
                self.last_command_direction = direction
                self.last_command_speed = speed
                
            elif command_type == 'turn_in_place':
                direction, speed = args
                self.bot.motor_controller.turn_in_place(direction, speed)
                self.current_movement_state = 'turning'
                self.last_command_time = current_time
                self.last_command_direction = direction
                self.last_command_speed = speed
                
            self.consecutive_errors = 0  # Reset error count on successful command
            
        except Exception as e:
            print(f"Motor command error: {e}")
            self.consecutive_errors += 1
            if self.consecutive_errors >= self.max_consecutive_errors:
                print("Too many motor errors, stopping")
                self.running = False
    
    def _get_robot_state(self):
        """Get current robot position and heading."""
        try:
            # Get IMU heading
            robot_heading = self.bot.imu.get_heading_rad()
            
            # Get ToF data for localization
            tof_pairs = self.bot.tof.get_localization_pairs(fresh=False)
            
            # Get position estimate
            robot_x, robot_y, _ = self.bot.localization.estimate_position(robot_heading, tof_pairs)
            
            return robot_x, robot_y, robot_heading
        except Exception as e:
            print(f"Error getting robot state: {e}")
            # Fallback if localization fails
            return config.field_center_x, config.field_center_y, 0.0
    
    def _is_near_boundary(self, x, y):
        """Check if robot is too close to field boundaries."""
        return (x < config.field_boundary_left + self.boundary_margin or
                x > config.field_boundary_right - self.boundary_margin or
                y < config.field_boundary_top + self.boundary_margin or
                y > config.field_boundary_bottom - self.boundary_margin)
    
    def _avoid_boundary(self, x, y, heading):
        """Move away from boundaries."""
        print(f"Near boundary at ({x:.0f}, {y:.0f}) - avoiding")
        
        # Calculate direction away from nearest boundary
        if x < config.field_boundary_left + self.boundary_margin:
            # Too close to left boundary - move right
            direction = 90  # degrees
        elif x > config.field_boundary_right - self.boundary_margin:
            # Too close to right boundary - move left
            direction = 270  # degrees
        elif y < config.field_boundary_top + self.boundary_margin:
            # Too close to top boundary - move down
            direction = 180  # degrees
        else:  # y > config.field_boundary_bottom - self.boundary_margin
            # Too close to bottom boundary - move up
            direction = 0  # degrees
        
        self._safe_motor_command('move_direction', direction, self.slow_speed_level)
    
    def _chase_ball(self, ball_angle, ball_distance):
        """Chase the ball based on angle and distance."""
        # Convert ball angle to movement direction
        # Ball angle is in radians, 0 = forward, clockwise
        # Move direction is in degrees, 0 = forward, clockwise
        direction_degrees = math.degrees(ball_angle)
        
        # Determine speed based on distance
        if ball_distance < self.close_distance:
            speed_level = self.slow_speed_level
            print(f"Close to ball ({ball_distance:.0f}px) - slow chase at {speed_level}")
        elif ball_distance > self.far_distance:
            speed_level = self.fast_speed_level
            print(f"Far from ball ({ball_distance:.0f}px) - fast chase at {speed_level}")
        else:
            speed_level = self.base_speed_level
            print(f"Medium distance to ball ({ball_distance:.0f}px) - normal chase at {speed_level}")
        
        # Move towards ball using safe command
        self._safe_motor_command('move_direction', direction_degrees, speed_level)
    
    def _search_for_ball(self):
        """Search for ball by turning when no ball is detected."""
        print("Searching for ball - turning")
        # Turn left to search using safe command
        self._safe_motor_command('turn_in_place', 'left', self.slow_speed_level)
    
    def stop(self):
        """Stop the agent and motors."""
        self.running = False
        self._safe_motor_command('stop')
        print("Blind Ball Chaser stopped")