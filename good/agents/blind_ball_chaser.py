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
        
        # Web server integration
        self.web_server = None
        
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
        """Single cycle of ball chasing logic - simplified to focus only on ball angle."""
        try:
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
                    # Simply chase the ball - no boundary checking for now
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
                self.last_command_direction = 0.0
                self.last_command_speed = 0.0
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
    
    def set_web_server(self, web_server):
        """Set reference to web server for data sharing"""
        self.web_server = web_server
    
    def _get_robot_heading(self):
        """Get current robot heading from IMU."""
        try:
            # Get IMU heading
            robot_heading = self.bot.imu.get_heading_rad()
            return robot_heading
        except Exception as e:
            print(f"Error getting robot heading: {e}")
            # Fallback if IMU fails
            return 0.0
    
    
    def _chase_ball(self, ball_angle, ball_distance):
        """Chase the ball based on angle and distance - simplified approach."""
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