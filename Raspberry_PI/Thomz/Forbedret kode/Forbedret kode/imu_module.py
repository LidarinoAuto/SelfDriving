# Filename: imu_module.py
# Module for handling IMU (gyro) data and related functions

import time
import math
from logging_utils import skriv_logg
import compass_module
import motor_control
# import mpu6050 # Uncomment if you use this module

# --- Configuration Constants ---
Kp_heading = 1.5
Kd_heading = 0.5
heading_tolerance = 5.0
base_speed_heading = 50
MAX_ROBOT_ROTATION_DEGPS_AT_BASE_SPEED = 37.80
MIN_INTERNAL_ROTATION_COMMAND = 50.0
MIN_ROBOT_ROTATION_DEGPS = (MIN_INTERNAL_ROTATION_COMMAND / base_speed_heading) * MAX_ROBOT_ROTATION_DEGPS_AT_BASE_SPEED if base_speed_heading != 0 else 0.0

# --- Global Variables ---
gyro_bias_z = 0.0
imu_initialized = False
gyro_bias_calibration_done = False

# --- Gyro Calibration Function ---
# Placeholder - replace with actual MPU6050 reading
def calibrate_gyro(samples=500):
    global gyro_bias_z, gyro_bias_calibration_done
    skriv_logg("Calibrating gyro bias. Keep robot completely still...")

    total_raw_z = 0.0
    read_count = 0

    # Placeholder: Replace with actual gyro reading loop
    skriv_logg("Gyro calibration placeholder: Simulating readings...")
    for i in range(samples):
         raw_z = 0.0 # Replace with: mpu6050.read_gyro_z_raw()
         total_raw_z += raw_z
         read_count += 1
         time.sleep(0.01)

    if read_count > 0:
        gyro_bias_z = total_raw_z / read_count
        skriv_logg(f"Gyro bias Z calibrated: {gyro_bias_z:.4f}")
        gyro_bias_calibration_done = True
        return True
    else:
        skriv_logg("Gyro calibration failed: No samples read.")
        gyro_bias_calibration_done = False
        return False

# --- Raw IMU Data Reading Function ---
# Placeholder - replace with actual MPU6050 reading
def read_imu_data():
    global gyro_bias_z, gyro_bias_calibration_done, imu_initialized
    if not imu_initialized or not gyro_bias_calibration_done:
        return 0.0

    try:
        # Placeholder: Replace with actual gyro reading
        raw_gyro_z = 0.0 # Replace with: mpu6050.read_gyro_z_raw()
        corrected_gyro_z = raw_gyro_z - gyro_bias_z
        return corrected_gyro_z # Return degrees/second

    except Exception as e:
        skriv_logg(f"Error reading IMU data: {e}")
        return 0.0

# --- Initialization Function ---
# Placeholder - replace with actual MPU6050 initialization
def init_imu_system():
    global imu_initialized
    skriv_logg("Initializing IMU system...")

    # Placeholder: Replace with actual MPU6050 initialization
    skriv_logg("IMU initialized successfully.")
    imu_initialized = True

    calibration_success = calibrate_gyro()

    if imu_initialized and gyro_bias_calibration_done:
         skriv_logg("IMU system initialization and calibration successful.")
         return True
    else:
         skriv_logg("IMU system initialization or calibration failed.")
         return False

# --- Heading Rotation Function (Absolute Heading) ---
def rotate_to_heading(heading_tracker, target_heading_degrees, timeout=20.0):
    global imu_initialized, gyro_bias_calibration_done
    if not imu_initialized or not gyro_bias_calibration_done or not compass_module.compass_initialized:
        skriv_logg("Cannot execute rotate_to_heading: IMU/Compass not initialized or calibrated.")
        motor_control.send_command(f"0.0 0.0 {0.0:.2f}\n")
        return False

    if heading_tracker is None:
        skriv_logg("Error: rotate_to_heading requires a valid HeadingTracker instance.")
        motor_control.send_command(f"0.0 0.0 {0.0:.2f}\n")
        return False

    target_heading_degrees = target_heading_degrees % 360.0
    if target_heading_degrees < 0:
        target_heading_degrees += 360

    start_rotation_time = time.time()

    heading_tracker.update()
    current_heading = heading_tracker.get_heading()

    # Static correction based on observed systematic offset (~-105 degrees)
    korrigert_heading = (current_heading - 105.0 + 360.0) % 360.0

    if current_heading == -1.0:
        skriv_logg("Error: Initial heading tracker reading failed in rotate_to_heading.")
        motor_control.send_command(f"0.0 0.0 {0.0:.2f}\n")
        return False

    error = ((target_heading_degrees - korrigert_heading + 540.0) % 360.0) - 180.0

    previous_heading = korrigert_heading
    last_time_pid = time.time()
    previous_error = error
    previous_smoothed_rate_of_change = 0.0

    skriv_logg(f"Executing initial rotation to {target_heading_degrees:.1f} (Error: {error:.1f})...")
    skriv_logg(f"Starting rotation (compass) to heading: {target_heading_degrees:.2f}")

    while (time.time() - start_rotation_time) < timeout:
        current_time_pid = time.time()
        dt = current_time_pid - last_time_pid
        last_time_pid = current_time_pid

        heading_tracker.update()
        current_heading = heading_tracker.get_heading()

        korrigert_heading = (current_heading - 105.0 + 360.0) % 360.0

        if current_heading == -1.0:
            skriv_logg("Warning: Heading tracker returned -1.0 during rotation. Skipping PID update.")
            motor_control.send_command(f"0.0 0.0 {0.0:.2f}\n")
            time.sleep(0.1)
            continue

        error = ((target_heading_degrees - korrigert_heading + 540.0) % 360.0) - 180.0

        if abs(error) <= heading_tolerance:
            skriv_logg(f"Target heading {target_heading_degrees:.2f} reached within tolerance {heading_tolerance:.2f}.")
            break

        error_change = error - previous_error
        alpha_derivative = 0.3
        raw_rate_of_change = (error_change / dt) if dt > 0 else 0.0
        smoothed_rate_of_change = alpha_derivative * raw_rate_of_change + (1 - alpha_derivative) * previous_smoothed_rate_of_change
        previous_smoothed_rate_of_change = smoothed_rate_of_change
        derivative_term = Kd_heading * smoothed_rate_of_change

        previous_error = error

        rotate_speed_command_internal_scale = (Kp_heading * error) + derivative_term
        rotate_speed_command_internal_scale = max(min(rotate_speed_command_internal_scale, base_speed_heading), -base_speed_heading)

        rotation_command_dps = 0.0
        if base_speed_heading != 0:
            rotation_command_dps = (rotate_speed_command_internal_scale / base_speed_heading) * MAX_ROBOT_ROTATION_DEGPS_AT_BASE_SPEED

        if abs(error) > heading_tolerance and abs(rotation_command_dps) > 0.0 and abs(rotation_command_dps) < MIN_ROBOT_ROTATION_DEGPS:
            rotation_command_dps = math.copysign(MIN_ROBOT_ROTATION_DEGPS, rotate_speed_command_internal_scale)
        elif abs(error) <= heading_tolerance:
             rotation_command_dps = 0.0

        command_to_send = f"{0.0:.2f} {0.0:.2f} {rotation_command_dps:.2f}\n"
        motor_control.send_command(command_to_send)
        time.sleep(0.05)

    motor_control.send_command(f"0.0 0.0 {0.0:.2f}\n")
    time.sleep(0.5)

    heading_tracker.update()
    korrigert_heading_final = (heading_tracker.get_heading() - 105.0 + 360.0) % 360.0
    final_error = ((target_heading_degrees - korrigert_heading_final + 540.0) % 360.0) - 180.0

    end_time = time.time()
    duration = end_time - start_rotation_time

    if abs(final_error) <= heading_tolerance:
        skriv_logg(f"Heading rotation to {target_heading_degrees:.2f} completed. Final heading: {korrigert_heading_final:.2f} in {duration:.2f}s.")
        return True
    elif (end_time - start_rotation_time) >= timeout:
        skriv_logg(f"Heading rotation to {target_heading_degrees:.2f} timed out after {duration:.2f}s. Final heading: {korrigert_heading_final:.2f}, Remaining error: {final_error:.2f}.")
        return False
    else:
        skriv_logg(f"Heading rotation to {target_heading_degrees:.2f} exited unexpectedly after {duration:.2f}s. Final heading: {korrigert_heading_final:.2f}, Remaining error: {final_error:.2f}.")
        return False

# --- Placeholder for rotate_by_gyro function ---
# Replace with actual gyro-based rotation logic
def rotate_by_gyro(angle_degrees, timeout=10.0):
    global imu_initialized, gyro_bias_calibration_done
    if not imu_initialized or not gyro_bias_calibration_done:
        skriv_logg("Cannot execute rotate_by_gyro: IMU not initialized or calibrated.")
        motor_control.send_command(f"0.0 0.0 {0.0:.2f}\n")
        return False

    skriv_logg(f"Executing placeholder rotate_by_gyro for {angle_degrees:.1f} degrees...")

    # Placeholder logic: Simulate rotation time
    abs_angle = abs(angle_degrees)
    estimated_time = abs_angle / MAX_ROBOT_ROTATION_DEGPS_AT_BASE_SPEED if MAX_ROBOT_ROTATION_DEGPS_AT_BASE_SPEED > 0 else 1.0
    simulated_duration = min(estimated_time * 1.5, timeout)

    simulated_rotation_speed = math.copysign(MAX_ROBOT_ROTATION_DEGPS_AT_BASE_SPEED * 0.5, angle_degrees)
    motor_control.send_command(f"0.0 0.0 {simulated_rotation_speed:.2f}\n")

    time.sleep(simulated_duration)

    motor_control.send_command("0.0 0.0 0.0\n")
    time.sleep(0.5)

    skriv_logg(f"Placeholder rotate_by_gyro completed (simulated).")
    return True

# Note: No if __name__ == "__main__": block here. Code is started from main.py.
