# Filename: imu_module.py
# Module for interfacing with the IMU (Gyroscope) sensor
# Includes bias calibration and upside-down compensation

import smbus  # For I2C communication
import time
import math  # Potentially useful for future extensions
import motor_control  # Required for sending rotation commands

# --- Configuration ---
IMU_ADDRESS = 0x68  # I2C address for MPU-6050 or compatible
PWR_MGMT_1 = 0x6B
GYRO_ZOUT_H = 0x47  # High-byte register for Z-axis gyro data

# Gyroscope scale factor for �250�/s setting
GYRO_SCALE_FACTOR = 250.0 / 32768.0

# --- Global Variables ---
bus = None
integrated_angle = 0.0
last_time = 0.0
gyro_bias_z = 0.0
bias_calibration_done = False


def init_imu():
    """
    Initializes IMU and performs bias calibration.
    Returns True if successful, False otherwise.
    """
    global bus, last_time, gyro_bias_z, bias_calibration_done
    print("Initializing IMU...")
    try:
        bus = smbus.SMBus(1)
        bus.write_byte_data(IMU_ADDRESS, PWR_MGMT_1, 0)
        time.sleep(0.1)
        # Bias calibration
        print("Calibrating gyro bias. Keep robot completely still...")
        total = 0.0
        count = 200
        for _ in range(count):
            total += read_raw_gyro_z()
            time.sleep(0.01)
        gyro_bias_z = total / count
        bias_calibration_done = True
        print(f"Gyro bias Z calibrated: {gyro_bias_z:.4f}")
        last_time = time.time()
        print("IMU initialized.")
        return True
    except FileNotFoundError:
        print("Error: I2C bus not found. Enable I2C in configuration.")
    except Exception as e:
        print(f"Error during IMU initialization: {e}")
    bus = None
    bias_calibration_done = False
    return False


def read_raw_gyro_z():
    """Reads raw Z-axis gyro data as signed 16-bit value."""
    if bus is None:
        return 0
    try:
        high = bus.read_byte_data(IMU_ADDRESS, GYRO_ZOUT_H)
        low = bus.read_byte_data(IMU_ADDRESS, GYRO_ZOUT_H + 1)
        raw = (high << 8) | low
        if raw > 32767:
            raw -= 65536
        return raw
    except Exception:
        return 0


def read_imu_data():
    """
    Returns compensated Z-axis angular velocity in �/s.
    """
    global gyro_bias_z, bias_calibration_done
    if bus is None or not bias_calibration_done:
        return 0.0
    raw = read_raw_gyro_z()
    corrected = raw - gyro_bias_z
    rate = corrected * GYRO_SCALE_FACTOR
    # Upside-down compensation
    return -rate


def rotate_by_gyro(target_angle_degrees):
    """
    Rotates robot by target_angle_degrees using IMU feedback.
    Positive: right turn. Negative: left turn.
    """
    global integrated_angle, last_time
    integrated_angle = 0.0
    last_time = time.time()

    if bus is None or not bias_calibration_done:
        print("Cannot execute rotation: IMU not ready.")
        return

    print(f"Starting rotation: {target_angle_degrees:.2f}�")
    # PID parameters
    Kp, Ki, Kd = 0.8, 0.0, 0.05
    integral_err = 0.0
    last_err = target_angle_degrees
    tolerance = 0.5
    base_speed = 18
    direction = 1 if target_angle_degrees > 0 else -1

    # Begin rotation command
    motor_control.send_command(f"0 0 {direction * base_speed}\n")

    while abs(target_angle_degrees - integrated_angle) > tolerance:
        now = time.time()
        dt = max(now - last_time, 1e-4)
        last_time = now
        rate = read_imu_data()
        integrated_angle += rate * dt

        # PID control
        error = target_angle_degrees - integrated_angle
        integral_err += error * dt
        derivative = (error - last_err) / dt
        last_err = error
        adjust = Kp * error + Ki * integral_err + Kd * derivative
        speed = max(min(abs(adjust), 100), 5)
        motor_control.send_command(f"0 0 {direction * speed}\n")
        time.sleep(0.01)

    motor_control.send_command("0 0 0\n")
    print("Rotation complete.")
