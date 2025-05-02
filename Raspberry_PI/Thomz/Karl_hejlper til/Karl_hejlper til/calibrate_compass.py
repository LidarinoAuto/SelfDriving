# calibrate_compass.py
"""
Calibrate compass by rotating full 360 degrees and logging raw values.
"""
import time
from compass_module import CompassModule
from logging_utils import setup_logger

logger = setup_logger('CalibrateCompass')
compass = CompassModule()

if __name__ == '__main__':
    try:
        logger.info("Starting compass calibration - rotate 360 degrees slowly")
        start = time.time()
        while time.time() - start < 60:
            heading = compass.read_heading()
            logger.debug(f"Raw heading: {heading}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        logger.info("Calibration complete")