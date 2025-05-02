# compass_module.py
"""
Compass module interface using smbus2.
"""
import smbus2
import math
from logging_utils import setup_logger
import config

class CompassModule:
    def __init__(self):
        self.bus = smbus2.SMBus(config.COMPASS_I2C_BUS)
        self.address = config.COMPASS_ADDRESS
        self.logger = setup_logger(self.__class__.__name__)

    def read_heading(self) -> float:
        """Return heading in degrees 0-360."""
        try:
            data = self.bus.read_i2c_block_data(self.address, 0x03, 6)
            x = self._twos_complement(data[0] << 8 | data[1], 16)
            z = self._twos_complement(data[2] << 8 | data[3], 16)
            y = self._twos_complement(data[4] << 8 | data[5], 16)
            heading = math.degrees(math.atan2(y, x))
            return heading + 360 if heading < 0 else heading
        except Exception:
            self.logger.exception("Failed to read compass")
            return float('nan')

    @staticmethod
    def _twos_complement(val, bits):
        return val - (1 << bits) if val & (1 << (bits - 1)) else val