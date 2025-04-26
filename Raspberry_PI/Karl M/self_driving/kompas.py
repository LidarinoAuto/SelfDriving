# kompas.py (oppdatert for offset)
import smbus
import time
import math
import os

QMC5883L_ADDRESS = 0x0d
bus = smbus.SMBus(1)

QMC5883L_CTRL1 = 0x09
QMC5883L_SET_RESET = 0x0B
QMC5883L_DATA = 0x00

offset_x = 0
offset_y = 0

def setup_compass():
    global offset_x, offset_y
    try:
        bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_SET_RESET, 0x01)
        time.sleep(0.1)
        bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_CTRL1, 0b00011101)
        time.sleep(0.1)
        print("QMC5883L initialisert.")

        # Last inn offset hvis fil finnes
        if os.path.exists("kompas_offset.txt"):
            with open("kompas_offset.txt", "r") as f:
                lines = f.readlines()
                if len(lines) >= 2:
                    offset_x = float(lines[0].strip())
                    offset_y = float(lines[1].strip())
                    print(f"Offset lastet: x = {offset_x:.2f}, y = {offset_y:.2f}")
    except Exception as e:
        print(f"Feil ved initiering av kompass: {e}")

def read_heading():
    try:
        data = bus.read_i2c_block_data(QMC5883L_ADDRESS, QMC5883L_DATA, 6)
        x_raw = (data[1] << 8) | data[0]
        y_raw = (data[3] << 8) | data[2]

        x = x_raw - 65536 if x_raw > 32767 else x_raw
        y = y_raw - 65536 if y_raw > 32767 else y_raw

        # Bruk offset
        x -= offset_x
        y -= offset_y

        heading_rad = math.atan2(y, x)
        heading_deg = math.degrees(heading_rad)

        if heading_deg < 0:
            heading_deg += 360

        return heading_deg
    except Exception as e:
        print(f"Feil ved lesing av kompass: {e}")
        return -1
