# Heading_Tracker.py
"""
Maintain desired heading using compass feedback and motor control.
"""
from compass_module import CompassModule
from motor_control import MotorController
import config
import time

class HeadingTracker:
    def __init__(self, target_heading: float):
        self.compass = CompassModule()
        self.motors = MotorController()
        self.target = target_heading

    def update(self):
        current = self.compass.read_heading()
        if current != current:
            return
        error = (self.target - current + 540) % 360 - 180
        if abs(error) < config.HEADING_THRESHOLD_DEGREES:
            self.motors.stop()
        else:
            turn_rate = max(min(error / 180, 1.0), -1.0)
            self.motors.set_speed(0.0, 0.0, turn_rate)

    def cleanup(self):
        self.motors.cleanup()