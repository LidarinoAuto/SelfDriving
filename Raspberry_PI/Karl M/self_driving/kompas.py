# kompas.py
import smbus
import time
import math

# Forutsetter en vanlig HMC5883L, LSM303, eller lignende magnetometer
DEVICE_ADDRESS = 0x1E  # Endre hvis ditt kompass har annen adresse
bus = smbus.SMBus(1)

def setup_compass():
    # Konfigurer kompasset
    bus.write_byte_data(DEVICE_ADDRESS, 0x00, 0x70)  # Configuration Register A
    bus.write_byte_data(DEVICE_ADDRESS, 0x01, 0xA0)  # Configuration Register B (gain)
    bus.write_byte_data(DEVICE_ADDRESS, 0x02, 0x00)  # Mode Register (continuous mode)
    time.sleep(0.5)

def read_raw_data(addr):
    high = bus.read_byte_data(DEVICE_ADDRESS, addr)
    low = bus.read_byte_data(DEVICE_ADDRESS, addr+1)
    value = ((high << 8) | low)
    if value > 32768:
        value = value - 65536
    return value

def get_heading():
    x = read_raw_data(0x03)
    z = read_raw_data(0x05)
    y = read_raw_data(0x07)

    heading_rad = math.atan2(y, x)
    heading_deg = math.degrees(heading_rad)
    if heading_deg < 0:
        heading_deg += 360
    return heading_deg
