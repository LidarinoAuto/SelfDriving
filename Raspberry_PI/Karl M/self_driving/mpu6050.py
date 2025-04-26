# mpu6050.py
import smbus
import time
import os

MPU6050_ADDR = 0x68
bus = smbus.SMBus(1)

offset_z = 0

def setup_mpu6050():
    global offset_z
    bus.write_byte_data(MPU6050_ADDR, 0x6B, 0x00)
    time.sleep(0.1)

    # Last inn offset hvis fil finnes
    if os.path.exists("gyro_offset.txt"):
        with open("gyro_offset.txt", "r") as f:
            offset_z = float(f.readline().strip())
            print(f"Lastet gyro offset: {offset_z:.5f}")

def read_gyro_z_raw():
    high = bus.read_byte_data(MPU6050_ADDR, 0x47)
    low = bus.read_byte_data(MPU6050_ADDR, 0x48)
    value = (high << 8) | low
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    return -(value / 131.0)

def read_gyro_z():
    high = bus.read_byte_data(MPU6050_ADDR, 0x47)
    low = bus.read_byte_data(MPU6050_ADDR, 0x48)
    value = (high << 8) | low
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    
    gyro_z = -(value / 131.0)
    gyro_z -= offset_z  # Fjern offset
    return gyro_z
