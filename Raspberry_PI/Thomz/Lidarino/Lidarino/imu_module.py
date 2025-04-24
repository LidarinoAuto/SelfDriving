import smbus
import time

bus = smbus.SMBus(1)
IMU_ADDRESS = 0x68
ROTATION_SPEED = 35

def init_imu():
    bus.write_byte_data(IMU_ADDRESS, 0x68, 0)
    time.sleep(0.1)

def read_word(adr):
    high = bus.read_byte_data(IMU_ADDRESS, adr)
    low = bus.read_byte_data(IMU_ADDRESS, adr + 1)
    return (high << 8) + low

def read_word_2c(adr):
    val = read_word(adr)
    return -((65535 - val) + 1) if val >= 0x8000 else val

def get_gyro_z():
    return read_word_2c(0x47) / 131.0

def rotate_by_gyro(target_angle):
    from motor_control import send_command
    integrated_angle = 0.0
    last_time = time.time()
    update_interval = 0.1
    last_update = time.time()
    send_command(f"0 0 {-ROTATION_SPEED}\n")

    while abs(integrated_angle) < abs(target_angle):
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        integrated_angle += get_gyro_z() * dt

        if abs(target_angle) - abs(integrated_angle) < 10:
            send_command(f"0 0 {-max(1, ROTATION_SPEED // 2)}\n")

        if current_time - last_update >= update_interval:
            print(f"Integrert vinkel: {integrated_angle:.2f}")
            last_update = current_time

        time.sleep(0.005)

    send_command("0 0 0\n")