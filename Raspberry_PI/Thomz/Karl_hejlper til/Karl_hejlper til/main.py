# main.py
"""
Main entrypoint for robot control loop.
"""
import time
from lidar_module import LidarModule
from ultrasound_module import UltrasoundModule
from compass_module import CompassModule
from imu_module import ImuModule
from motor_control import MotorController
from Heading_Tracker import HeadingTracker
from logging_utils import setup_logger

logger = setup_logger('Main')

if __name__ == '__main__':
    lidar = LidarModule()
    ultrasound = UltrasoundModule()
    compass = CompassModule()
    imu = ImuModule()
    motors = MotorController()
    tracker = HeadingTracker(target_heading=0.0)

    lidar.start()
    try:
        while True:
            readings = {
                'lidar_mm': lidar.get_distance(),
                'ultrasound_cm': ultrasound.read_distance(),
                'heading_deg': compass.read_heading(),
                'gyro_dps': imu.read_gyro()
            }
            logger.info(f"Readings: {readings}")
            tracker.update()
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("Shutting down")
    finally:
        lidar.stop()
        ultrasound.cleanup()
        motors.cleanup()
        tracker.cleanup()