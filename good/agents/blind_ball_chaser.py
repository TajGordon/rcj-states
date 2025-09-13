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
        
        # For the new rotate-then-forward logic, we don't need complex angle/distance checks
        # Just check if enough time has passed since last command
        print(f"      ✅ Time threshold met - movement allowed")
        return True
    
    def _safe_motor_command(self, command_type, *args):
        """Safely execute motor commands with error handling."""
        print(f"   🔧 _safe_motor_command called: {command_type} with args {args}")
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
                
            elif command_type == 'rotate':
                speed, direction = args
                print(f"   🔄 ROTATING: {direction} at speed {speed}")
                if direction == 'right':
                    # Rotate right (clockwise) - left motors forward, right motors backward
                    self.bot.motor_controller.turn_in_place('right', speed)
                else:  # left
                    # Rotate left (counter-clockwise) - right motors forward, left motors backward
                    self.bot.motor_controller.turn_in_place('left', speed)
                self.current_movement_state = 'turning'
                self.last_command_time = current_time
                self.last_command_direction = direction
                self.last_command_speed = speed
                print(f"   ✅ Rotate command sent successfully")
                
            elif command_type == 'set_speeds':
                speeds = args[0]  # speeds is the first argument
                print(f"   🎮 SETTING DIRECT MOTOR SPEEDS:")
                print(f"      Speeds: {speeds}")
                self.bot.motor_controller.set_motor_speeds(speeds)
                self.current_movement_state = 'moving'
                self.last_command_time = current_time
                self.last_command_direction = 0.0  # Not applicable for direct speeds
                self.last_command_speed = max(abs(speed) for speed in speeds) / 1000  # Approximate speed level
                print(f"   ✅ Direct motor speeds set successfully")
                
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
        """Chase the ball using soccer_bot coordinate system and motor calculation."""
        # Get ball position from camera
        ball_x, ball_y = self.bot.camera.get_ball_position()
        
        if ball_x is None or ball_y is None:
            print(f"   ❌ No ball position available")
            return
        
        # DEBUG: Print ball detection info
        print(f"🎯 BALL DETECTED:")
        print(f"   Ball position: ({ball_x}, {ball_y})")
        print(f"   Ball angle (rad): {ball_angle:.3f}")
        print(f"   Ball angle (deg): {math.degrees(ball_angle):.1f}°")
        print(f"   Ball distance: {ball_distance:.0f}px")
        
        # Use soccer_bot coordinate system
        frame_center_x = 320  # Camera center X
        frame_center_y = 320  # Camera center Y
        
        # Calculate error from ball position to center of frame (soccer_bot method)
        error_x = ball_x - frame_center_x
        error_y = frame_center_y - ball_y  # Note: Y is inverted in image coordinates
        
        error_x_norm = error_x / frame_center_x
        error_y_norm = error_y / frame_center_y
        
        print(f"   📐 COORDINATE SYSTEM:")
        print(f"      Error X: {error_x}px (norm: {error_x_norm:.3f})")
        print(f"      Error Y: {error_y}px (norm: {error_y_norm:.3f})")
        print(f"      Direction: {'RIGHT' if error_x > 0 else 'LEFT' if error_x < 0 else 'CENTER'}")
        print(f"      Ball position: ({ball_x}, {ball_y})")
        print(f"      Frame center: ({frame_center_x}, {frame_center_y})")
        
        # Simple logic: rotate to face ball, then move forward
        angle_threshold = 0.05  # ~3 degrees - close enough to consider "facing" the ball
        
        if abs(error_x_norm) > angle_threshold:
            # Need to rotate to face the ball
            print(f"   🔄 ROTATING TO FACE BALL")
            self._rotate_to_face_ball(error_x_norm, ball_distance)
        else:
            # Facing the ball, move forward
            print(f"   ➡️  MOVING FORWARD TOWARD BALL")
            self._move_forward_toward_ball(ball_distance)
    
    def _rotate_to_face_ball(self, error_x_norm, ball_distance):
        """Rotate the robot to face the ball directly."""
        # Determine rotation direction and speed
        if error_x_norm > 0:
            # Ball is to the right, rotate right (clockwise)
            rotation_speed = min(0.3, abs(error_x_norm) * 2.0)  # Scale rotation speed
            print(f"      Rotating RIGHT (clockwise) at speed {rotation_speed:.2f}")
            print(f"      DEBUG: About to call _safe_motor_command('rotate', {rotation_speed}, 'right')")
            self._safe_motor_command('rotate', rotation_speed, 'right')
        else:
            # Ball is to the left, rotate left (counter-clockwise)
            rotation_speed = min(0.3, abs(error_x_norm) * 2.0)  # Scale rotation speed
            print(f"      Rotating LEFT (counter-clockwise) at speed {rotation_speed:.2f}")
            print(f"      DEBUG: About to call _safe_motor_command('rotate', {rotation_speed}, 'left')")
            self._safe_motor_command('rotate', rotation_speed, 'left')
    
    def _move_forward_toward_ball(self, ball_distance):
        """Move the robot forward toward the ball."""
        # Determine forward speed based on distance
        if ball_distance < self.close_distance:
            forward_speed = self.slow_speed_level
            print(f"      Moving SLOW forward (close to ball) at speed {forward_speed:.2f}")
        elif ball_distance > self.far_distance:
            forward_speed = self.fast_speed_level
            print(f"      Moving FAST forward (far from ball) at speed {forward_speed:.2f}")
        else:
            forward_speed = self.base_speed_level
            print(f"      Moving forward at speed {forward_speed:.2f}")
        
        print(f"      DEBUG: About to call _safe_motor_command('move_direction', 0.0, {forward_speed})")
        self._safe_motor_command('move_direction', 0.0, forward_speed)  # 0.0 = straight forward
    
    def _calculate_motor_speeds_soccer_bot_style(self, error_x_norm, error_y_norm, ball_distance):
        """Calculate motor speeds using soccer_bot method adapted for your hardware"""
        # Use YOUR hardware max speed (not soccer_bot's)
        max_speed = self.bot.motor_controller.max_speed  # 30,000,000 for your setup
        
        # Soccer bot control parameters (reduced turn sensitivity to prevent spinning)
        kp_turn = 0.5  # Reduced from 2.0 to prevent aggressive turning
        kp_forward = 0.8
        turn_threshold = 0.1
        tight_turn_factor = 0.3
        pure_turn_threshold = 0.4
        nonlinear_turn_power = 0.5
        
        # Determine speed based on distance (adapted for your hardware)
        # Your speed levels are 1.0-3.0, so scale them appropriately
        if ball_distance < self.close_distance:
            speed_multiplier = self.slow_speed_level / 4.0  # 1.0/4.0 = 0.25 (25% of max)
        elif ball_distance > self.far_distance:
            speed_multiplier = self.fast_speed_level / 4.0  # 3.0/4.0 = 0.75 (75% of max)
        else:
            speed_multiplier = self.base_speed_level / 4.0  # 2.0/4.0 = 0.5 (50% of max)
        
        # Apply speed multiplier to max_speed
        effective_max_speed = max_speed * speed_multiplier
        
        print(f"   🎮 MOTOR CALCULATION:")
        print(f"      Speed multiplier: {speed_multiplier:.2f}")
        print(f"      Effective max speed: {effective_max_speed//1000}k")
        
        # Calculate turning adjustment based on horizontal ball position (soccer_bot method)
        # TEMPORARY: Force straight movement to test if motors work correctly
        if True:  # abs(error_x_norm) < turn_threshold:
            turn_adjustment = 0
            is_turning = False
            print(f"      Turn mode: FORCED_STRAIGHT (testing mode)")
        else:
            # Use nonlinear turning response
            error_sign = 1 if error_x_norm > 0 else -1
            error_magnitude = abs(error_x_norm)
            nonlinear_error = error_sign * (error_magnitude ** nonlinear_turn_power)
            turn_adjustment = nonlinear_error * effective_max_speed * kp_turn
            is_turning = True
            
            if abs(error_x_norm) > pure_turn_threshold:
                print(f"      Turn mode: PURE_TURN (error > {pure_turn_threshold})")
            else:
                print(f"      Turn mode: TIGHT_TURN")
        
        # Base forward movement speed
        if is_turning:
            if abs(error_x_norm) > pure_turn_threshold:
                forward_speed = effective_max_speed * kp_forward * 0.1
            else:
                forward_speed = effective_max_speed * kp_forward * tight_turn_factor
        else:
            forward_speed = effective_max_speed * kp_forward
        
        print(f"      Forward speed: {forward_speed//1000}k")
        print(f"      Turn adjustment: {turn_adjustment//1000}k")
        print(f"      Turn direction: {'RIGHT' if turn_adjustment > 0 else 'LEFT' if turn_adjustment < 0 else 'STRAIGHT'}")
        
        # Calculate individual motor speeds (soccer_bot method adapted for your hardware)
        # Motors: [27-back left, 28-back right, 30-front left, 26-front right] (from your config)
        # Front-left (30) and back-left (27) motors are inverted due to hardware orientation
        
        # Back-left motor (27): forward movement (INVERTED) + turn adjustment (inverted for left motor)
        back_left_speed = -(forward_speed + turn_adjustment)
        
        # Back-right motor (28): forward movement - turn adjustment
        back_right_speed = forward_speed - turn_adjustment
        
        # Front-left motor (30): forward movement (INVERTED) + turn adjustment (inverted for left motor)
        front_left_speed = -(forward_speed + turn_adjustment)
        
        # Front-right motor (26): forward movement - turn adjustment
        front_right_speed = forward_speed - turn_adjustment
        
        # Clip speeds to max limits
        speeds = [back_left_speed, back_right_speed, front_left_speed, front_right_speed]
        speeds = [int(max(-effective_max_speed, min(effective_max_speed, speed))) for speed in speeds]
        
        print(f"      Final speeds: BL:{speeds[0]//1000}k BR:{speeds[1]//1000}k FL:{speeds[2]//1000}k FR:{speeds[3]//1000}k")
        
        # Debug: Check if all motors are going in the same direction (which would cause spinning)
        if all(speed > 0 for speed in speeds) or all(speed < 0 for speed in speeds):
            print(f"      ⚠️  WARNING: All motors same direction - will cause spinning!")
        elif speeds[0] * speeds[1] > 0 and speeds[2] * speeds[3] > 0:
            print(f"      ⚠️  WARNING: Left/right motors same direction - will cause spinning!")
        
        return speeds
    
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