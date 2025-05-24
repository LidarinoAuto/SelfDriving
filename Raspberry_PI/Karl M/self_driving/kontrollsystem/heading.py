# heading.py
import math
import time
from sensorer import kompas
from sensorer import mpu6050

# Konstanter
KOMPASS_JUSTERING = 270
GYRO_WEIGHT = 0.50
COMPASS_WEIGHT = 1.0 - GYRO_WEIGHT

class HeadingTracker:
    def __init__(self):
        self.last_time = time.time()
        self.fused_heading = 0

    def setup(self):
        kompas.setup_compass()
        mpu6050.setup_mpu6050()
        heading = kompas.read_heading()
        if heading != -1:
            self.fused_heading = (heading + KOMPASS_JUSTERING) % 360
        else:
            self.fused_heading = 0

    def update(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        gyro_z = mpu6050.read_gyro_z()
        self.fused_heading = (self.fused_heading + gyro_z * dt) % 360

        compass_heading = kompas.read_heading()
        if compass_heading != -1:
            compass_heading = (compass_heading + KOMPASS_JUSTERING) % 360
            delta = ((compass_heading - self.fused_heading + 540) % 360) - 180
            self.fused_heading = (self.fused_heading + delta * (1.0 - GYRO_WEIGHT)) % 360

        return self.fused_heading

    def get_heading(self):
        return self.fused_heading
