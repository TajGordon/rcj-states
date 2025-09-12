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
        
        # Movement parameters
        self.base_speed_level = 6  # Base speed when chasing ball
        self.slow_speed_level = 3  # Speed when close to ball
        self.fast_speed_level = 8  # Speed when far from ball
        
        # Distance thresholds (in pixels)
        self.close_distance = 80   # Close to ball
        self.far_distance = 200    # Far from ball
        
        # Boundary safety margins (in mm)
        self.boundary_margin = 100  # Stay 100mm away from field boundaries
        
        # Movement state
        self.last_ball_angle = 0.0
        self.last_ball_distance = 0.0
        self.no_ball_count = 0
        self.max_no_ball_count = 10  # Stop after 10 cycles without ball
        
    def run(self, target_goal):
        """
        Main agent loop - blindly chase the ball while avoiding boundaries.
        
        Args:
            target_goal: 'blue' or 'yellow' - which goal to defend (not used in blind chaser)
        """
        self.running = True
        print(f"Blind Ball Chaser started - Target goal: {target_goal}")
        
        try:
            while self.running:
                self._chase_ball_cycle()
                time.sleep(0.05)  # 20 Hz main loop
                
        except KeyboardInterrupt:
            print("Blind Ball Chaser interrupted")
        finally:
            self.stop()
    
    def _chase_ball_cycle(self):
        """Single cycle of ball chasing logic."""
        # Get current robot position and heading
        robot_x, robot_y, robot_heading = self._get_robot_state()
        
        # Get ball information
        ball_detected = self.bot.camera.is_ball_detected()
        
        if ball_detected:
            self.no_ball_count = 0
            
            # Get ball data
            ball_angle = self.bot.camera.get_ball_angle()
            ball_distance = self.bot.camera.get_ball_distance_from_center()
            
            # Store for reference
            self.last_ball_angle = ball_angle
            self.last_ball_distance = ball_distance
            
            # Check if we're too close to boundaries
            if self._is_near_boundary(robot_x, robot_y):
                self._avoid_boundary(robot_x, robot_y, robot_heading)
            else:
                # Chase the ball
                self._chase_ball(ball_angle, ball_distance)
        else:
            # No ball detected
            self.no_ball_count += 1
            
            if self.no_ball_count > self.max_no_ball_count:
                # Stop if no ball for too long
                self.bot.motor_controller.stop_motors()
                print("No ball detected for too long - stopping")
            else:
                # Use last known ball position or search
                if self.last_ball_angle != 0.0:
                    print(f"No ball - using last known angle: {math.degrees(self.last_ball_angle):.1f}°")
                    self._chase_ball(self.last_ball_angle, self.last_ball_distance)
                else:
                    # Search by turning
                    self._search_for_ball()
    
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
        except:
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
        
        self.bot.motor_controller.move_direction(direction, self.slow_speed_level)
    
    def _chase_ball(self, ball_angle, ball_distance):
        """Chase the ball based on angle and distance."""
        # Convert ball angle to movement direction
        # Ball angle is in radians, 0 = forward, clockwise
        # Move direction is in degrees, 0 = forward, clockwise
        direction_degrees = math.degrees(ball_angle)
        
        # Determine speed based on distance
        if ball_distance < self.close_distance:
            speed_level = self.slow_speed_level
            print(f"Close to ball ({ball_distance:.0f}px) - slow chase")
        elif ball_distance > self.far_distance:
            speed_level = self.fast_speed_level
            print(f"Far from ball ({ball_distance:.0f}px) - fast chase")
        else:
            speed_level = self.base_speed_level
            print(f"Medium distance to ball ({ball_distance:.0f}px) - normal chase")
        
        # Move towards ball
        self.bot.motor_controller.move_direction(direction_degrees, speed_level)
    
    def _search_for_ball(self):
        """Search for ball by turning when no ball is detected."""
        print("Searching for ball - turning")
        # Turn left to search
        self.bot.motor_controller.turn_in_place('left', self.slow_speed_level)
    
    def stop(self):
        """Stop the agent and motors."""
        self.running = False
        self.bot.motor_controller.stop_motors()
        print("Blind Ball Chaser stopped")