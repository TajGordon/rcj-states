from imu import IMU
from camera import Camera
from tof_stuff import ToFArray
from motor_controller import MotorController
from localization import Localization
from agent import Agent

class Bot:
    def __init__(self):
        self.camera = Camera()
        self.localization = Localization()
        self.motor_controller = MotorController()
        self.imu = IMU()
        self.tof = ToFArray()
        self.agent = Agent()

    def run(self):
        pass

    def stop(self):
        pass

