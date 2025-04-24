import smbus
import time
import math

bus = smbus.SMBus(1)
HMC5883L_ADDRESS = 0x0d

# Init
bus.write_byte_data(HMC5883L_ADDRESS, 0x00, 0x70)
bus.write_byte_data(HMC5883L_ADDRESS, 0x01, 0xA0)
bus.write_byte_data(HMC5883L_ADDRESS, 0x02, 0x00)

def read_compass():
    data = bus.read_i2c_block_data(HMC5883L_ADDRESS, 0x03, 6)
    x = data[0] << 8 | data[1]
    z = data[2] << 8 | data[3]
    y = data[4] << 8 | data[5]
    x = x - 65536 if x > 32767 else x
    y = y - 65536 if y > 32767 else y
    heading_rad = math.atan2(y, x)
    heading_deg = math.degrees(heading_rad)
    if heading_deg < 0:
        heading_deg += 360
    return heading_deg

while True:
    heading = read_compass()
    print(f"Kompassretning: {heading:.2f}�")
    time.sleep(0.5)
