# imu_module.py
"""
Gyroscope (and accelerometer) sensor interface using I2C (e.g., MPU-6050).
"""
import smbus2
import time
from logging_utils import setup_logger
import config

class ImuModule:
    def __init__(self):
        self.bus = smbus2.SMBus(config.GYRO_I2C_BUS)
        self.address = config.GYRO_ADDRESS
        self.logger = setup_logger(self.__class__.__name__)

    def read_gyro(self) -> tuple:
        """Read angular rate in degrees/sec for x, y, z axes."""
        try:
            data = self.bus.read_i2c_block_data(self.address, 0x43, 6)
            x_rate = (self._twos_complement(data[0] << 8 | data[1], 16) / 131.0)
            y_rate = (self._twos_complement(data[2] << 8 | data[3], 16) / 131.0)
            z_rate = (self._twos_complement(data[4] << 8 | data[5], 16) / 131.0)
            return x_rate, y_rate, z_rate
        except Exception:
            self.logger.exception("Failed to read IMU")
            return float('nan'), float('nan'), float('nan')

    @staticmethod
    def _twos_complement(val, bits):
        return val - (1 << bits) if val & (1 << (bits - 1)) else val