# mpu6050.py
import smbus
import time

MPU6050_ADDR = 0x68
bus = smbus.SMBus(1)

def setup_mpu6050():
    # Vekk opp MPU6050 fra sleep mode
    bus.write_byte_data(MPU6050_ADDR, 0x6B, 0x00)
    time.sleep(0.1)

def read_gyro_z():
    # Gyro Z er på register 0x47 og 0x48
    high = bus.read_byte_data(MPU6050_ADDR, 0x47)
    low = bus.read_byte_data(MPU6050_ADDR, 0x48)
    value = (high << 8) | low
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    
    # Skaler til grader/sekund (MPU6050 default 250dps range)
    gyro_z = value / 131.0
    return gyro_z
