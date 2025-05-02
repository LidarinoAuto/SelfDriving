# Filename: imu_module.py
# Module for handling IMU (gyro) data and related functions

import time
import math
from logging_utils import skriv_logg
import compass_module
import motor_control
# Import your actual mpu6050 module
import mpu6050 # Assumes your file is named mpu6050.py

# --- Configuration Constants ---
Kp_heading = 1.5 # Proportional gain for Heading Control
Kd_heading = 0.5 # Derivative gain for Heading Control
# Ki_heading = 0.0 # Integral gain (not used in this version)

heading_tolerance = 5.0 # Degrees. Tolerance for rotate_to_heading completion

GYRO_ROTATION_COMMAND_VALUE = 50.0 # Internal command value for gyro rotation

base_speed_heading = 50 # Max rotation speed command scale

# >>> UPDATE THIS VALUE WITH THE RESULT FROM YOUR CALIBRATION SCRIPT (calibrate_rotate.py)! <<<
MAX_ROBOT_ROTATION_DEGPS_AT_BASE_SPEED = 37.80 # Max rotation speed in deg/s at base_speed_heading

MIN_INTERNAL_ROTATION_COMMAND = 50.0 # Minimum command value to send to motors for rotation
MIN_ROBOT_ROTATION_DEGPS = (MIN_INTERNAL_ROTATION_COMMAND / base_speed_heading) * MAX_ROBOT_ROTATION_DEGPS_AT_BASE_SPEED if base_speed_heading != 0 else 0.0

GYRO_ROTATION_TOLERANCE = 2.0 # Degrees. Tolerance for rotate_by_gyro completion


# --- Global Variables ---
gyro_bias_z = 0.0 # Will be set by calibrate_gyro
imu_initialized = False # Status for MPU6050 initialization
gyro_bias_calibration_done = False # Status for gyro bias calibration

# --- Gyro Calibration Function ---
# This function uses read_gyro_z_raw() from your mpu6050.py
def calibrate_gyro(samples=500):
    """
    Calibrates the Z-axis gyro bias by averaging raw readings while the robot is still.
    Assumes mpu6050 module is available and initialized.
    Sets the global gyro_bias_z and gyro_bias_calibration_done flag.
    """
    global gyro_bias_z, gyro_bias_calibration_done
    skriv_logg("Calibrating gyro bias. Keep robot completely still...")

    if not imu_initialized:
        skriv_logg("MPU6050 not initialized. Cannot calibrate gyro.")
        gyro_bias_calibration_done = False
        return False

    total_raw_z = 0.0
    read_count = 0

    skriv_logg(f"Reading {samples} raw gyro Z samples for bias calibration...")
    for i in range(samples):
         try:
             raw_z_deg_s = mpu6050.read_gyro_z_raw()
             if raw_z_deg_s is not None:
                 total_raw_z += raw_z_deg_s
                 read_count += 1
             else:
                 skriv_logg(f"Warning: Failed to read MPU6050 raw gyro during calibration sample {i}.")

             time.sleep(0.01)
         except Exception as e:
             skriv_logg(f"Error reading MPU6050 during calibration sample {i}: {e}")
             pass

    if read_count > 0:
        gyro_bias_z = total_raw_z / read_count
        skriv_logg(f"Gyro bias Z calibrated: {gyro_bias_z:.4f} deg/s")
        gyro_bias_calibration_done = True
        return True
    else:
        skriv_logg("Gyro calibration failed: No valid samples read.")
        gyro_bias_calibration_done = False
        return False


# --- IMU Data Reading Function ---
# This function uses read_gyro_z_raw() from your mpu6050.py and applies imu_module's bias
def read_imu_data():
    """
    Reads raw IMU data (gyro Z in deg/s) from mpu6050.py and applies imu_module's calculated bias.
    Returns the corrected gyro Z value (degrees per second), or 0.0 on error/not calibrated.
    """
    global gyro_bias_z, gyro_bias_calibration_done, imu_initialized
    if not imu_initialized or not gyro_bias_calibration_done:
        return 0.0

    try:
        raw_gyro_z_deg_s = mpu6050.read_gyro_z_raw()

        if raw_gyro_z_deg_s is not None:
            corrected_gyro_z = raw_gyro_z_deg_s - gyro_bias_z
            return corrected_gyro_z
        else:
            return 0.0

    except Exception as e:
        skriv_logg(f"Error reading IMU data from mpu6050.py: {e}")
        return 0.0


# --- IMU Initialization Function ---
# This function uses setup_mpu6050() from your mpu6050.py
def init_imu_system():
    """
    Initializes the IMU (Gyro) system by calling setup_mpu6050() and performs gyro calibration.
    Returns True if initialization and calibration are successful, False otherwise.
    Sets the global imu_initialized flag.
    """
    global imu_initialized

    skriv_logg("Initializing IMU system...")
    if mpu6050.setup_mpu6050():
       skriv_logg("MPU6050 initialized successfully via mpu6050.py.")
       imu_initialized = True
    else:
       skriv_logg("MPU6050 initialization failed via mpu6050.py.")
       imu_initialized = False
       return False

    calibration_success = calibrate_gyro()

    if imu_initialized and gyro_bias_calibration_done:
         skriv_logg("IMU system initialization and calibration successful.")
         return True
    else:
         skriv_logg("IMU system initialization or calibration failed.")
         return False


# --- Heading Rotation Function (Absolute Heading using Compass) ---
def rotate_to_heading(heading_tracker, target_heading_degrees, timeout=20.0):
    """
    Rotates the robot to a target absolute heading using the compass via HeadingTracker.
    Requires a HeadingTracker instance.
    Returns True if rotation is completed within tolerance/timeout, False otherwise.
    """
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

    rotation_timeout = 20.0
    start_rotation_time = time.time()

    heading_tracker.update()
    current_heading = heading_tracker.get_heading() # Heading is corrected by get_heading()

    if current_heading == -1.0:
        skriv_logg("Error: Initial heading tracker reading failed in rotate_to_heading.")
        motor_control.send_command(f"0.0 0.0 {0.0:.2f}\n")
        return False

    error = ((target_heading_degrees - current_heading + 540.0) % 360.0) - 180.0

    previous_heading = current_heading
    last_time_pid = time.time()
    previous_error = error
    previous_smoothed_rate_of_change = 0.0

    skriv_logg(f"Executing initial rotation to {target_heading_degrees:.1f} (Error: {error:.1f})...")
    skriv_logg(f"Starting rotation (compass) to heading: {target_heading_degrees:.2f}")

    while (time.time() - start_rotation_time) < rotation_timeout:
        current_time_pid = time.time()
        dt = current_time_pid - last_time_pid
        last_time_pid = current_time_pid

        heading_tracker.update()
        current_heading = heading_tracker.get_heading() # Heading is corrected by get_heading()

        if current_heading == -1.0:
            skriv_logg("Warning: Heading tracker returned -1.0 during rotation. Skipping PID update.")
            motor_control.send_command(f"0.0 0.0 {0.0:.2f}\n")
            time.sleep(0.1)
            continue

        error = ((target_heading_degrees - current_heading + 540.0) % 360.0) - 180.0

        skriv_logg(f"PID Debug - Fused Heading: {current_heading:.2f} , Error: {error:.2f} ")

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
    final_heading = heading_tracker.get_heading() # Heading is corrected by get_heading()
    final_error = ((target_heading_degrees - final_heading + 540.0) % 360.0) - 180.0

    end_time = time.time()
    duration = end_time - start_rotation_time

    if abs(final_error) <= heading_tolerance:
        skriv_logg(f"Heading rotation to {target_heading_degrees:.2f} completed. Final heading: {final_heading:.2f} in {duration:.2f}s.")
        return True
    elif (end_time - start_rotation_time) >= rotation_timeout:
        skriv_logg(f"Heading rotation to {target_heading_degrees:.2f} timed out after {duration:.2f}s. Final heading: {final_heading:.2f} , Remaining error: {final_error:.2f} .")
        return False
    else:
        skriv_logg(f"Heading rotation to {target_heading_degrees:.2f} exited unexpectedly after {duration:.2f}s. Final heading: {final_heading:.2f} , Remaining error: {final_error:.2f} .")
        return False

# --- Gyro-based Rotation Function (Relative Angle) ---
def rotate_by_gyro(angle_degrees, timeout=10.0):
    """
    Rotates the robot by a specified angle (in degrees) using only gyro data.
    Positive angle for clockwise, negative for counter-clockwise.
    Uses gyro integration to track the rotated angle.
    Returns True if rotation is completed within tolerance/timeout, False otherwise.
    """
    global imu_initialized, gyro_bias_calibration_done
    if not imu_initialized or not gyro_bias_calibration_done:
        skriv_logg("Cannot execute rotate_by_gyro: IMU not initialized or calibrated.")
        motor_control.send_command(f"0.0 0.0 {0.0:.2f}\n")
        return False

    skriv_logg(f"Executing rotate_by_gyro for {angle_degrees:.1f} degrees...")

    start_time = time.time()
    last_loop_time = time.time()
    total_gyro_rotation = 0.0

    rotation_direction = math.copysign(1.0, angle_degrees)
    rotation_speed_command = GYRO_ROTATION_COMMAND_VALUE * rotation_direction

    target_rotation = abs(angle_degrees)

    motor_control.send_command(f"0.0 0.0 {rotation_speed_command:.2f}\n")

    while (time.time() - start_time) < timeout:
        current_loop_time = time.time()
        dt = current_loop_time - last_loop_time
        last_loop_time = current_loop_time

        # Read corrected gyro data in degrees/second using imu_module's read_imu_data
        gyro_rate = read_imu_data() # This now calls mpu6050.read_gyro_z_raw() and applies bias

        if dt > 0:
            gyro_delta_rotation = gyro_rate * dt
            total_gyro_rotation += gyro_delta_rotation

        # Check if the target rotation has been reached
        # We check if the absolute value of the total accumulated rotation is close to the target angle.
        if abs(total_gyro_rotation) >= target_rotation - GYRO_ROTATION_TOLERANCE and abs(total_gyro_rotation) <= target_rotation + GYRO_ROTATION_TOLERANCE:
             skriv_logg(f"Target gyro rotation {target_rotation:.2f} degrees reached within tolerance {GYRO_ROTATION_TOLERANCE:.2f}. Total integrated rotation: {total_gyro_rotation:.2f}")
             break

        # Optional: Add a small delay
        # time.sleep(0.01)

    skriv_logg("Stopping gyro rotation...")
    motor_control.send_command(f"0.0 0.0 {0.0:.2f}\n")
    time.sleep(0.5)

    end_time = time.time()
    duration = end_time - start_time

    # Report result
    if abs(total_gyro_rotation) >= target_rotation - GYRO_ROTATION_TOLERANCE and abs(total_gyro_rotation) <= target_rotation + GYRO_ROTATION_TOLERANCE:
        skriv_logg(f"Gyro rotation of {angle_degrees:.2f} degrees completed. Total integrated rotation: {total_gyro_rotation:.2f} in {duration:.2f}s.")
        return True
    elif (end_time - start_time) >= timeout:
        skriv_logg(f"Gyro rotation to {angle_degrees:.2f} degrees timed out after {duration:.2f}s. Total integrated rotation: {total_gyro_rotation:.2f}.")
        return False
    else:
        skriv_logg(f"Gyro rotation to {angle_degrees:.2f} degrees exited unexpectedly after {duration:.2f}s. Total integrated rotation: {total_gyro_rotation:.2f}.")
        return False

# Note: No if __name__ == "__main__": block here. Code is started from main.py.