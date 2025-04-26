# kompas.py (for QMC5883L)
import smbus
import time
import math

QMC5883L_ADDRESS = 0x0d  # Standard adresse
bus = smbus.SMBus(1)

QMC5883L_CTRL1 = 0x09
QMC5883L_SET_RESET = 0x0B
QMC5883L_DATA = 0x00

def setup_compass():
    try:
        bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_SET_RESET, 0x01)
        time.sleep(0.1)
        bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_CTRL1, 0b00011101)
        time.sleep(0.1)
        print("QMC5883L initialisert.")
    except Exception as e:
        print(f"Feil ved initiering av kompass: {e}")

def read_heading():
    try:
        data = bus.read_i2c_block_data(QMC5883L_ADDRESS, QMC5883L_DATA, 6)

        x_raw = (data[1] << 8) | data[0]
        y_raw = (data[3] << 8) | data[2]
        z_raw = (data[5] << 8) | data[4]

        x = x_raw - 65536 if x_raw > 32767 else x_raw
        y = y_raw - 65536 if y_raw > 32767 else y_raw
        z = z_raw - 65536 if z_raw > 32767 else z_raw

        heading_rad = math.atan2(y, x)
        heading_deg = math.degrees(heading_rad)

        if heading_deg < 0:
            heading_deg += 360

        return heading_deg
    except Exception as e:
        print(f"Feil ved lesing av kompass: {e}")
        return -1
