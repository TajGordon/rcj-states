import board
import time
import busio
import config
import signal
import sys
from imu import IMU
from camera import Camera
from motor_controller import MotorController
from agents.blind_ball_chaser import Agent

class Bot:
    def __init__(self):
        # Create single I2C instance for all components
        self.i2c = busio.I2C(board.SCL, board.SDA)
        
        # Initialize components with shared I2C instance
        self.motor_controller = MotorController(i2c=self.i2c)
        self.camera = Camera(enable_frame_queue=True, detection_mode='enhanced')
        self.imu = IMU(i2c=self.i2c)
        # Note: ToF and Localization not needed for simple ball following
        self.agent = Agent(bot=self)
        
        # Verify motor controller is ready
        if not self.motor_controller.are_motors_ready():
            print("WARNING: Not all motors initialized properly")
        
        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)

    def run(self):
        try:
            # Start camera
            self.camera.start()
            
            # Wait for camera to initialize and verify it's working
            print("Waiting for camera to initialize...")
            time.sleep(2)  # Give camera time to start
            
            # Verify camera is running and can detect
            if not self.camera.is_running:
                print("ERROR: Camera failed to start")
                return
            
            print("Camera started successfully")
            
            # Run the main agent logic with motor updates
            self._run_with_motor_updates()
            
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.stop()
    
    def _run_with_motor_updates(self):
        """Run agent with regular motor data updates"""
        import threading
        import time
        
        # Flag to control the motor update thread
        self.motor_update_running = True
        
        def motor_update_loop():
            """Background thread to update motor data"""
            while self.motor_update_running:
                try:
                    self.motor_controller.update_motor_data()
                    time.sleep(0.01)  # Update at 100Hz
                except Exception as e:
                    print(f"Motor update error: {e}")
                    time.sleep(0.1)
        
        # Start motor update thread
        motor_thread = threading.Thread(target=motor_update_loop, daemon=True)
        motor_thread.start()
        
        try:
            # Run the main agent logic
            self.agent.run(config.target_goal)
        finally:
            # Stop motor update thread
            self.motor_update_running = False

    def stop(self):
        """Stop all components and cleanup"""
        try:
            # Stop motor update thread
            if hasattr(self, 'motor_update_running'):
                self.motor_update_running = False
            
            self.camera.stop()
            self.motor_controller.stop_motors()
        except:
            pass

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        self.stop()
        sys.exit(0)


if __name__ == "__main__":
    bot = Bot()
    bot.run()

