import board
import busio
import config
import signal
import sys
from imu import IMU
from camera import Camera
from tof_stuff import ToFArray
from motor_controller import MotorController
from localization import Localization
from agents.blind_ball_chaser import Agent

class Bot:
    def __init__(self):
        # Create single I2C instance for all components
        self.i2c = busio.I2C(board.SCL, board.SDA)
        
        # Initialize components with shared I2C instance
        self.motor_controller = MotorController(i2c=self.i2c)
        self.camera = Camera()
        self.imu = IMU(i2c=self.i2c)
        self.tof = ToFArray(self.i2c)
        self.localization = Localization()
        self.agent = Agent(bot=self)
        
        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)

    def run(self):
        try:
            # Start camera
            self.camera.start()
            
            # Run the main agent logic
            self.agent.run(config.target_goal)
            
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.stop()

    def stop(self):
        """Stop all components and cleanup"""
        try:
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

