# calibrate_rotate.py
"""
Rotate robot in place for calibration of rotation speed vs heading change.
"""
import time
from motor_control import MotorController
from compass_module import CompassModule
from logging_utils import setup_logger

logger = setup_logger('CalibrateRotate')
motors = MotorController()
compass = CompassModule()

if __name__ == '__main__':
    try:
        logger.info("Starting rotation calibration")
        motors.set_speed(0.5, 0.0, -0.5)
        start_heading = compass.read_heading()
        start_time = time.time()
        time.sleep(5)
        end_heading = compass.read_heading()
        elapsed = time.time() - start_time
        delta = ((end_heading - start_heading + 540) % 360) - 180
        logger.info(f"Rotated {delta}° in {elapsed:.2f}s")
    finally:
        motors.stop()
        motors.cleanup()
