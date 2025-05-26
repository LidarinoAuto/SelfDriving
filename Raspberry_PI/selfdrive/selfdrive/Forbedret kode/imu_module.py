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
Kp_heading = 1.2 # Proportional gain for Heading Control
Kd_heading = 0.5 # Derivative gain for Heading Control
# Ki_heading = 0.0 # Integral gain (not used in this version)
heading_tolerance = 5.0 # Degrees. Tolerance for rotate_to_heading completion
GYRO_ROTATION_COMMAND_VALUE = 50.0 # Internal command value for gyro rotation
base_speed_heading = 50 # Max rotation speed command scale
# >>> UPDATE THIS VALUE WITH THE RESULT FROM YOUR CALIBRATION SCRIPT (calibrate_rotate.py)! <<<
MAX_ROBOT_ROTATION_DEGPS_AT_BASE_SPEED = 37.80 # Max rotation speed in deg/s at base_speed_heading
# Minimum rotation speed in degrees per second to overcome friction/inertia
# This is calculated based on a minimum internal command value (e.g., 50)
MIN_INTERNAL_ROTATION_COMMAND = 50.0 # Minimum command value to send to motors for rotation
MIN_ROBOT_ROTATION_DEGPS = (MIN_INTERNAL_ROTATION_COMMAND / base_speed_heading) * MAX_ROBOT_ROTATION_DEGPS_AT_BASE_SPEED

# Global variables for IMU data and calibration
gyro_bias_z = 0.0
imu_initialized = False

# --- Initialization ---
def init_imu_system():
    """
    Initializes the MPU6050 and calibrates the gyro bias.
    Returns True if successful, False otherwise.
    """
    global imu_initialized

    skriv_logg("Initializing IMU system...")
    try:
        if mpu6050.setup_mpu6050():
            skriv_logg("MPU6050 initialized successfully via mpu6050.py.")
            calibrate_gyro()
            imu_initialized = True
            skriv_logg("IMU system initialization and calibration successful.")
            return True
        else:
            skriv_logg("Failed to initialize MPU6050.")
            imu_initialized = False
            return False
    except Exception as e:
        skriv_logg(f"Error during IMU system initialization: {e}")
        imu_initialized = False
        return False


def calibrate_gyro(num_samples=500):
    """
    Calibrates the gyro Z-axis bias by averaging readings while stationary.
    """
    global gyro_bias_z
    skriv_logg(f"Calibrating gyro bias. Keep robot completely still...")
    z_sum = 0.0
    skriv_logg(f"Reading {num_samples} raw gyro Z samples for bias calibration...")

    # Ensure MPU6050 is set up before reading
    if not imu_initialized and not mpu6050.setup_mpu6050():
         skriv_logg("MPU6050 not set up for calibration!")
         return False # Cannot calibrate if sensor is not set up

    # Read samples and calculate sum
    start_time = time.time()
    for i in range(num_samples):
        try:
            z_sum += mpu6050.read_gyro_z_raw() # Use raw reading for calibration
            time.sleep(0.01) # Small delay between readings
        except Exception as e:
            skriv_logg(f"Error reading gyro during calibration sample {i}: {e}")
            # Decide if you want to stop calibration or continue with fewer samples
            continue # Continue for now, but log the error

    end_time = time.time()
    duration = end_time - start_time
    skriv_logg(f"Finished reading {num_samples} gyro samples in {duration:.2f}s.")

    if num_samples > 0:
        gyro_bias_z = z_sum / num_samples
        skriv_logg(f"Gyro bias Z calibrated: {gyro_bias_z:.4f} deg/s")
        return True
    else:
        skriv_logg("No gyro samples read for calibration.")
        gyro_bias_z = 0.0
        return False


# --- Data Reading ---
def read_imu_data():
    """
    Reads corrected gyro Z data.
    Returns corrected gyro Z value in degrees per second, or None if IMU not initialized.
    """
    if not imu_initialized:
        #skriv_logg("IMU not initialized. Cannot read data.")
        return None

    try:
        # Read raw gyro Z value
        raw_gyro_z_deg_s = mpu6050.read_gyro_z_raw() # Use raw reading

        # Apply calibrated bias
        # --- FIX: INVERT GYRO SIGN FOR UPSIDE-DOWN MOUNTING ---
        # Based on observation of rotate_by_gyro logs, the gyro Z sign appears inverted.
        # We flip the sign of the corrected reading here.
        corrected_gyro_z = -(raw_gyro_z_deg_s - gyro_bias_z)
        # Alternative way to write the same:
        # corrected_gyro_z = gyro_bias_z - raw_gyro_z_deg_s
        # --- END FIX ---


        # Return corrected gyro Z value
        return corrected_gyro_z

    except Exception as e:
        skriv_logg(f"Error reading IMU data: {e}")
        return None

# --- Rotation Functions ---

# Assume HeadingTracker instance is passed to rotation functions
# and is updated externally in a loop (e.g., in main.py)

def rotate_to_heading(heading_tracker, target_heading_degrees, timeout=10.0):
    """
    Rotates the robot to face a specific absolute heading using PID control.
    Requires an updated HeadingTracker instance.
    Args:
        heading_tracker: An instance of HeadingTracker providing fused heading.
        target_heading_degrees: The desired absolute heading (0-360).
        timeout: Maximum time to attempt rotation in seconds.
    Returns:
        True if rotation completed successfully within tolerance and timeout, False otherwise.
    """
    if heading_tracker is None:
        skriv_logg("HeadingTracker not provided to rotate_to_heading.")
        return False

    start_time = time.time()
    last_error = 0.0 # For derivative term
    # last_integral = 0.0 # For integral term (if Ki > 0)

    skriv_logg(f"Starting rotation (compass) to heading: {target_heading_degrees:.2f} ")

    while time.time() - start_time < timeout:
        current_heading = heading_tracker.get_heading() # Get the latest fused heading (already offset adjusted if implemented in HeadingTracker)

        if current_heading == -1.0:
            skriv_logg("HeadingTracker not initialized or providing invalid heading.")
            time.sleep(0.1) # Wait a bit before trying again
            continue

        # Calculate the shortest angle difference (error)
        # This handles wrapping around 0/360 degrees
        error = target_heading_degrees - current_heading
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        # PID calculations (simplified PD)
        # Proportional term
        P_term = Kp_heading * error

        # Derivative term
        D_term = Kd_heading * (error - last_error) # / dt - dt is assumed ~constant per loop

        # Integral term (uncomment and implement if Ki > 0)
        # last_integral += error * dt # Requires actual dt
        # I_term = Ki_heading * last_integral

        # Combine terms (PD control)
        pid_output = P_term + D_term # + I_term # Include I_term if used

        # --- DEBUG LOG ---
        skriv_logg(f"PID Debug - Fused Heading: {current_heading:.2f} , Error: {error:.2f} ")
        # --- END DEBUG LOG ---

        # Check if target heading is reached within tolerance
        if abs(error) <= heading_tolerance:
            skriv_logg(f"Target heading {target_heading_degrees:.2f} reached within tolerance {heading_tolerance:.2f} during PID loop.")
            break # Exit the loop as target is reached

        # Scale PID output based on max rotation speed
        max_pid_output = Kp_heading * 180 + Kd_heading * 180 # Very rough estimate
        rotation_speed_deg_s = pid_output # Assume PID output is already in deg/s or proportional to it. If not, scale it here.

        # Map degrees/s to internal motor command scale (-base_speed_heading to +base_speed_heading)
        # internal_rotation_command = (rotation_speed_deg_s / MAX_ROBOT_ROTATION_DEGPS_AT_BASE_SPEED) * base_speed_heading

        rotation_command_value = pid_output # Use PID output directly as the rotation command value scale

        # Let's apply the minimum threshold on the internal command value scale
        if abs(rotation_command_value) > 0 and abs(rotation_command_value) < MIN_INTERNAL_ROTATION_COMMAND:
             # Apply minimum command in the direction of the error
             rotation_command_value = math.copysign(MIN_INTERNAL_ROTATION_COMMAND, rotation_command_value)
             # skriv_logg(f"Applying min rotation command: {rotation_command_value}")


        # Clamp the command value to prevent sending values outside the expected range (-base_speed_heading to +base_speed_heading)
        rotation_command_value = max(-base_speed_heading, min(base_speed_heading, rotation_command_value))


        # Send rotation command to motors. Assuming format "left_speed right_speed rotation_speed\n"
        # For pure rotation, left_speed and right_speed are typically 0.
        motor_control.send_command(f"0.0 0.0 {rotation_command_value:.2f}\n")

        # Update for derivative term
        last_error = error

        # Add a small delay to allow the robot to respond and sensor to update
        time.sleep(0.05) # Adjust sleep time as needed for your sensor update rate

    # Stop the robot after the loop finishes
    skriv_logg("Stopping heading rotation...")
    motor_control.send_command(f"0.0 0.0 {0.0:.2f}\n")
    time.sleep(0.5) # Wait a bit for robot to physically stop

    end_time = time.time()
    duration = end_time - start_time

    # Check final heading to report success/failure
    final_heading = heading_tracker.get_heading() # Get final heading after stopping
    if final_heading == -1.0:
        skriv_logg("Failed to get final heading after rotation.")
        # Report failure based on timeout if heading is invalid
        if (end_time - start_time) >= timeout:
             skriv_logg(f"Heading rotation to {target_heading_degrees:.2f} timed out after {duration:.2f}s with invalid final heading.")
             return False
        else:
             # If loop broke due to tolerance but final heading is invalid, something is wrong.
             skriv_logg(f"Heading rotation to {target_heading_degrees:.2f} loop broke but final heading is invalid.")
             return False

    final_error = target_heading_degrees - final_heading
    if final_error > 180:
        final_error -= 360
    elif final_error < -180:
        final_error += 360


    # Report result
    if abs(error) <= heading_tolerance: # error is the error when the loop broke
         skriv_logg(f"Heading rotation to {target_heading_degrees:.2f} completed successfully (tolerance met in loop). Final heading: {final_heading:.2f} in {duration:.2f}s. Final error: {final_error:.2f} .")
         return True
    elif (end_time - start_time) >= timeout:
        skriv_logg(f"Heading rotation to {target_heading_degrees:.2f} timed out after {duration:.2f}s. Final heading: {final_heading:.2f}. Final error: {final_error:.2f}.")
        return False
    else:
         # Should not happen if loop exits only by timeout or tolerance met
         skriv_logg(f"Heading rotation to {target_heading_degrees:.2f} ended unexpectedly. Final heading: {final_heading:.2f}. Final error: {final_error:.2f}.")
         return False
def rotate_by_gyro(angle_degrees, timeout=5.0):
    """
    Rotates the robot by a specific relative angle using integrated gyro data.
    Note: Gyro integration accumulates error over time. Best for small, quick turns.
    Args:
        angle_degrees: The relative angle to rotate (positive for CW, negative for CCW, assuming standard convention after gyro sign fix).
        timeout: Maximum time to attempt rotation in seconds.
    Returns:
        True if rotation completed successfully within tolerance and timeout, False otherwise.
    """
    if not imu_initialized:
        skriv_logg("IMU not initialized. Cannot perform gyro rotation.")
        return False

    start_time = time.time()
    last_time = time.time()
    total_gyro_rotation = 0.0 # Integrated rotation in degrees
    # GYRO_ROTATION_TOLERANCE = 2.0 # Degrees tolerance for stopping

    # Use the same heading_tolerance for consistency if it makes sense for relative turns
    GYRO_ROTATION_TOLERANCE = heading_tolerance # Use the same tolerance as for heading control

    skriv_logg(f"Executing rotate_by_gyro for {angle_degrees:.2f} degrees...")

    # Determine rotation direction
    direction_sign = math.copysign(1.0, angle_degrees)
    target_rotation = abs(angle_degrees)
    rotation_command_value = direction_sign * GYRO_ROTATION_COMMAND_VALUE

    motor_control.send_command(f"0.0 0.0 {rotation_command_value:.2f}\n")

    # Integrate gyro data until target angle is reached
    while abs(total_gyro_rotation) < target_rotation - GYRO_ROTATION_TOLERANCE and (time.time() - start_time) < timeout:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        gyro_z_deg_s = read_imu_data() # Get corrected gyro Z

        if gyro_z_deg_s is not None:
            # Integrate rotation rate over time
            total_gyro_rotation += gyro_z_deg_s * dt

        # --- DEBUG LOG ---
        # skriv_logg(f"Gyro Rotate Debug - Gyro Z: {gyro_z_deg_s:.2f}, dt: {dt:.4f}, Total Rotation: {total_gyro_rotation:.2f}")
        # --- END DEBUG LOG ---

        # Check if rotation is close to target (this check is now mainly for logging/breaking early if needed)
        # The loop condition `abs(total_gyro_rotation) < target_rotation - GYRO_ROTATION_TOLERANCE`

        # The original check was:
        # if abs(total_gyro_rotation) >= target_rotation - GYRO_ROTATION_TOLERANCE and abs(total_gyro_rotation) <= target_rotation + GYRO_ROTATION_TOLERANCE:

    # Stop rotation after loop finishes
    skriv_logg("Stopping gyro rotation...")
    motor_control.send_command(f"0.0 0.0 {0.0:.2f}\n")
    time.sleep(0.5) # Wait a bit for robot to physically stop

    end_time = time.time()
    duration = end_time - start_time

    # Report result
    # Check if the magnitude of total rotation is within the tolerance of the target magnitude.
    if abs(total_gyro_rotation) >= target_rotation - GYRO_ROTATION_TOLERANCE:
        skriv_logg(f"Gyro rotation of {angle_degrees:.2f} degrees completed. Total integrated rotation: {total_gyro_rotation:.2f} in {duration:.2f}s.")
        return True
    elif (end_time - start_time) >= timeout:
        skriv_logg(f"Gyro rotation to {angle_degrees:.2f} degrees timed out after {duration:.2f}s. Total integrated rotation: {total_gyro_rotation:.2f}.")
        return False
    else:
         skriv_logg(f"Gyro rotation to {angle_degrees:.2f} ended unexpectedly. Total integrated rotation: {total_gyro_rotation:.2f}.")
         return False
