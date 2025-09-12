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
        print(f"🤖 Blind Ball Chaser started - Target goal: {target_goal}")
        print(f"📊 Movement parameters: base={self.base_speed_level}, slow={self.slow_speed_level}, fast={self.fast_speed_level}")
        print(f"📏 Movement thresholds:")
        print(f"   Min angle change: {math.degrees(self.min_angle_change):.1f}°")
        print(f"   Min distance change: {self.min_distance_change}px")
        print(f"   Min command interval: {self.min_command_interval}s")
        print(f"   Distance thresholds: close={self.close_distance}px, far={self.far_distance}px")
        
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
                
                print(f"\n🔍 CHASE CYCLE - Ball detected!")
                print(f"   Current state: {self.current_movement_state}")
                print(f"   Last command time: {time.time() - self.last_command_time:.2f}s ago")
                
                # Check if movement is needed (avoid micro-adjustments)
                should_move = self._should_move(ball_angle, ball_distance)
                print(f"   Should move: {should_move}")
                
                if should_move:
                    # Simply chase the ball - no boundary checking for now
                    self._chase_ball(ball_angle, ball_distance)
                else:
                    print(f"   ⏸️  Movement skipped (micro-adjustment threshold not met)")
                
                # Store for reference
                self.last_ball_angle = ball_angle
                self.last_ball_distance = ball_distance
            else:
                # No ball detected
                self.no_ball_count += 1
                print(f"\n❌ NO BALL DETECTED (count: {self.no_ball_count}/{self.max_no_ball_count})")
                
                if self.no_ball_count > self.max_no_ball_count:
                    # Stop if no ball for too long
                    self._safe_motor_command('stop')
                    print("   🛑 No ball detected for too long - stopping")
                else:
                    # Use last known ball position or search
                    if self.last_ball_angle != 0.0:
                        print(f"   🔄 Using last known angle: {math.degrees(self.last_ball_angle):.1f}°")
                        self._chase_ball(self.last_ball_angle, self.last_ball_distance)
                    else:
                        # Search by turning
                        print(f"   🔍 Searching for ball...")
                        self._search_for_ball()
                        
        except Exception as e:
            print(f"❌ Error in chase ball cycle: {e}")
            self.consecutive_errors += 1
            if self.consecutive_errors >= self.max_consecutive_errors:
                print("🚨 Too many consecutive errors, stopping")
                self.running = False
    
    def _should_move(self, ball_angle, ball_distance):
        """Check if movement is needed based on thresholds to avoid micro-adjustments."""
        current_time = time.time()
        
        print(f"   📏 MOVEMENT THRESHOLD CHECK:")
        
        # Check minimum time interval
        time_since_last = current_time - self.last_command_time
        print(f"      Time since last command: {time_since_last:.2f}s (min: {self.min_command_interval}s)")
        if time_since_last < self.min_command_interval:
            print(f"      ❌ Too soon since last command")
            return False
        
        # Check if angle change is significant enough
        angle_change = abs(ball_angle - self.last_command_direction)
        angle_change_deg = math.degrees(angle_change)
        print(f"      Angle change: {angle_change_deg:.1f}° (min: {math.degrees(self.min_angle_change):.1f}°)")
        if angle_change < self.min_angle_change:
            print(f"      ❌ Angle change too small")
            return False
        
        # Check if distance change is significant enough
        distance_change = abs(ball_distance - self.last_ball_distance)
        print(f"      Distance change: {distance_change:.0f}px (min: {self.min_distance_change}px)")
        if distance_change < self.min_distance_change:
            print(f"      ❌ Distance change too small")
            return False
        
        print(f"      ✅ All thresholds met - movement allowed")
        return True
    
    def _safe_motor_command(self, command_type, *args):
        """Safely execute motor commands with error handling."""
        try:
            current_time = time.time()
            
            if command_type == 'stop':
                print(f"   🛑 STOPPING MOTORS")
                self.bot.motor_controller.stop_motors()
                self.current_movement_state = 'stopped'
                self.last_command_time = current_time
                self.last_command_direction = 0.0
                self.last_command_speed = 0.0
                print("   ✅ Motors stopped successfully")
                
            elif command_type == 'move_direction':
                direction, speed = args
                print(f"   🎮 CALLING MOTOR CONTROLLER:")
                print(f"      move_direction({direction:.3f} rad, {speed})")
                
                # Call motor controller
                self.bot.motor_controller.move_direction(direction, speed)
                
                # Update state
                self.current_movement_state = 'moving'
                self.last_command_time = current_time
                self.last_command_direction = direction
                self.last_command_speed = speed
                
                print(f"   ✅ Motor command sent successfully")
                print(f"   📊 STATE UPDATE: {self.current_movement_state}, speed={speed}, dir={math.degrees(direction):.1f}°")
                
            elif command_type == 'turn_in_place':
                direction, speed = args
                print(f"   🔄 TURNING IN PLACE: {direction} at speed {speed}")
                self.bot.motor_controller.turn_in_place(direction, speed)
                self.current_movement_state = 'turning'
                self.last_command_time = current_time
                self.last_command_direction = direction
                self.last_command_speed = speed
                print(f"   ✅ Turn command sent successfully")
                
            self.consecutive_errors = 0  # Reset error count on successful command
            
        except Exception as e:
            print(f"   ❌ MOTOR COMMAND ERROR: {e}")
            self.consecutive_errors += 1
            if self.consecutive_errors >= self.max_consecutive_errors:
                print("   🚨 Too many motor errors, stopping agent")
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
        # DEBUG: Print ball detection info
        print(f"🎯 BALL DETECTED:")
        print(f"   Ball angle (rad): {ball_angle:.3f}")
        print(f"   Ball angle (deg): {math.degrees(ball_angle):.1f}°")
        print(f"   Ball distance: {ball_distance:.0f}px")
        
        # Ball angle is already in radians from camera (0=forward, clockwise)
        # Motor controller expects direction in radians (0=forward, π/2=right, π=back, 3π/2=left)
        # So we can use ball_angle directly!
        direction_rad = ball_angle
        
        # Determine speed based on distance
        if ball_distance < self.close_distance:
            speed_level = self.slow_speed_level
            print(f"   📍 CLOSE to ball ({ball_distance:.0f}px) - SLOW chase at speed {speed_level}")
        elif ball_distance > self.far_distance:
            speed_level = self.fast_speed_level
            print(f"   📍 FAR from ball ({ball_distance:.0f}px) - FAST chase at speed {speed_level}")
        else:
            speed_level = self.base_speed_level
            print(f"   📍 MEDIUM distance to ball ({ball_distance:.0f}px) - NORMAL chase at speed {speed_level}")
        
        # DEBUG: Print movement command
        print(f"   🚀 MOVEMENT COMMAND:")
        print(f"      Direction: {direction_rad:.3f} rad ({math.degrees(direction_rad):.1f}°)")
        print(f"      Speed: {speed_level}")
        print(f"      Expected movement: {self._describe_direction(direction_rad)}")
        
        # Move towards ball using safe command (pass radians, not degrees!)
        self._safe_motor_command('move_direction', direction_rad, speed_level)
    
    def _describe_direction(self, angle_rad):
        """Describe the direction in human-readable terms"""
        angle_deg = math.degrees(angle_rad)
        if angle_deg < 22.5 or angle_deg >= 337.5:
            return "FORWARD"
        elif angle_deg < 67.5:
            return "FORWARD-RIGHT"
        elif angle_deg < 112.5:
            return "RIGHT"
        elif angle_deg < 157.5:
            return "BACK-RIGHT"
        elif angle_deg < 202.5:
            return "BACK"
        elif angle_deg < 247.5:
            return "BACK-LEFT"
        elif angle_deg < 292.5:
            return "LEFT"
        else:
            return "FORWARD-LEFT"
    
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