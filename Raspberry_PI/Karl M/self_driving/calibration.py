# calibration.py
import time
import mpu6050
import kompas
import motorsignal

gyro_offset = 0
compass_offset_x = 0
compass_offset_y = 0

def calibrate_gyro():
    print("Starter gyro-kalibrering...")
    mpu6050.setup_mpu6050()

    samples = 500
    total = 0

    for _ in range(samples):
        gz = mpu6050.read_gyro_z_raw()
        total += gz
        time.sleep(0.005)

    global gyro_offset
    gyro_offset = total / samples
    print(f"Gyro offset målt: {gyro_offset:.5f} grader/s")

    with open("gyro_offset.txt", "w") as f:
        f.write(f"{gyro_offset}\n")
    print("Gyro-offset lagret til gyro_offset.txt.")

def calibrate_compass():
    print("Starter kompass-kalibrering...")
    kompas.setup_compass()

    rotation_speed = 30  # grader/s
    target_angle = 360

    min_x = min_y = 32767
    max_x = max_y = -32768

    integrated_angle = 0
    last_time = time.time()

    # Start rotasjon 360° mot venstre
    motorsignal.send_movement_command(0, 0, rotation_speed)

    start_time = time.time()

    while abs(integrated_angle) < target_angle and (time.time() - start_time) < 10:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        gz = mpu6050.read_gyro_z_raw()
        integrated_angle += abs(gz * dt)

        x, y = kompas.read_raw_xy()
        min_x = min(min_x, x)
        max_x = max(max_x, x)
        min_y = min(min_y, y)
        max_y = max(max_y, y)

        time.sleep(0.01)

    motorsignal.send_movement_command(0, 0, 0.0)
    time.sleep(1)

    # Snur tilbake
    integrated_angle = 0
    motorsignal.send_movement_command(0, 0, -rotation_speed)

    last_time = time.time()
    start_time = time.time()

    while abs(integrated_angle) < target_angle and (time.time() - start_time) < 10:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        gz = mpu6050.read_gyro_z_raw()
        integrated_angle += abs(gz * dt)

        x, y = kompas.read_raw_xy()
        min_x = min(min_x, x)
        max_x = max(max_x, x)
        min_y = min(min_y, y)
        max_y = max(max_y, y)

        time.sleep(0.01)

    motorsignal.send_movement_command(0, 0, 0.0)
    time.sleep(1)

    global compass_offset_x, compass_offset_y
    compass_offset_x = (min_x + max_x) / 2
    compass_offset_y = (min_y + max_y) / 2

    print(f"Kompass offset målt: x={compass_offset_x:.2f}, y={compass_offset_y:.2f}")

    with open("kompas_offset.txt", "w") as f:
        f.write(f"{compass_offset_x}\n")
        f.write(f"{compass_offset_y}\n")
    print("Kompass-offset lagret til kompas_offset.txt.")

def full_calibration():
    calibrate_gyro()
    calibrate_compass()
    print("Full kalibrering ferdig!")
