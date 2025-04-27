# Filename: imu_module.py
# Module for interfacing with the IMU (Gyroscope) sensor
# Includes bias calibration and basic proportional rotation control.

import smbus      # For I2C communication
import time
# Removed math import as it's not explicitly used in this simplified version

import motor_control # Import motor_control module


# --- Configuration ---
# IMPORTANT: Verify these settings for YOUR specific IMU chip!
# Common for MPU-6050:
IMU_ADDRESS = 0x68      # I2C address (0x68 or 0x69)
PWR_MGMT_1 = 0x6B       # Power Management Register 1
GYRO_ZOUT_H = 0x47      # High byte of Z-axis gyro data

# Gyroscope scale factor (adjust based on your IMU configuration)
# Example for +/- 250 degrees/second range:
GYRO_SCALE_FACTOR = 250.0 / 32768.0

# --- Global Variables ---
bus = None
integrated_angle = 0.0  # Integrated angle for current rotation
last_time = 0.0         # Time of the last IMU reading for integration
gyro_bias_z = 0.0       # Calculated Z-axis gyro bias
bias_calibration_done = False # Flag
imu_initialized = False # Added initialization flag


# --- Initialization ---
# Updated to set global flag and return status
def init_imu():
    """
    Initializes the IMU sensor (I2C) and performs gyro bias calibration.
    Requires the robot to be stationary during calibration.
    Returns True if initialization and calibration succeed, False otherwise.
    """
    global bus, last_time, gyro_bias_z, bias_calibration_done, imu_initialized

    print("Initializing IMU...")
    try:
        bus = smbus.SMBus(1) # Use bus 1 on Raspberry Pi

        # Wake up IMU (command depends on chip, MPU-6050 uses PWR_MGMT_1 = 0)
        # IMPORTANT: Check if this command is correct for your IMU!
        bus.write_byte_data(IMU_ADDRESS, PWR_MGMT_1, 0)
        time.sleep(0.1) # Give sensor time to start

        # --- Bias Calibration ---
        print("Calibrating gyro bias. Keep robot completely still...") # Updated print
        calibration_readings = 200 # Number of readings for calibration
        total_gyro_z = 0.0

        # Read raw gyro data multiple times while stationary
        for i in range(calibration_readings):
            raw_z = read_raw_gyro_z() # Read raw data
            total_gyro_z += raw_z
            time.sleep(0.01) # Small delay

        # Calculate average bias
        gyro_bias_z = total_gyro_z / calibration_readings
        bias_calibration_done = True # Set flag
        imu_initialized = True # Set initialized flag
        print(f"Gyro bias Z calibrated: {gyro_bias_z:.4f}")
        print("IMU initialized successfully.") # Success message

        # Set the initial time after calibration for integration
        last_time = time.time()
        return True # Indicate success

    except FileNotFoundError:
        print("Error: I2C bus not found. Make sure I2C is enabled in Raspberry Pi configuration.")
        bus = None
        bias_calibration_done = False
        imu_initialized = False
        return False # Indicate failure
    except Exception as e:
        print(f"Error during IMU initialization: {e}")
        bus = None
        bias_calibration_done = False
        imu_initialized = False
        return False # Indicate failure


# Helper function to read raw Z-axis gyro data
# IMPORTANT: Adjust register addresses and byte logic for YOUR specific IMU chip!
def read_raw_gyro_z():
    """Reads raw Z-axis gyroscope data from the IMU."""
    global bus
    if bus is None:
        return 0 # Return 0 if bus is not initialized or error occurred earlier

    try:
        # Read high and low byte for Z-axis gyro
        # IMPORTANT: Verify these register addresses for YOUR IMU!
        gyro_z_h = bus.read_byte_data(IMU_ADDRESS, GYRO_ZOUT_H)
        gyro_z_l = bus.read_byte_data(IMU_ADDRESS, GYRO_ZOUT_H + 1)

        # Combine bytes to a signed 16-bit integer
        raw_value = (gyro_z_h << 8) | gyro_z_l
        if raw_value > 32767: # Handle negative numbers (2's complement)
            raw_value -= 65536

        return raw_value

    except Exception:
        # Return 0 on error - this might cause inaccurate integration if reads fail mid-turn
        # print(f"Warning: Error reading raw gyro data: {e}") # Optional debug print
        return 0


# --- Data Reading and Processing ---
def read_imu_data():
    """
    Reads processed IMU data (Z-axis angular velocity in degrees/sec)
    after bias correction, scaling, and orientation compensation.
    Returns 0.0 if IMU is not initialized or calibrated.
    """
    global gyro_bias_z, bias_calibration_done, imu_initialized

    # Only process if IMU is initialized and calibrated
    if not imu_initialized or not bias_calibration_done:
        return 0.0

    raw_gyro_z = read_raw_gyro_z() # Read raw data

    # Apply Bias Correction
    corrected_gyro_z = raw_gyro_z - gyro_bias_z

    # Apply Scale Factor (convert raw to deg/sec)
    angular_velocity_z = corrected_gyro_z * GYRO_SCALE_FACTOR

    # Apply Upside-Down Compensation
    # Negate Z-axis velocity if IMU is mounted upside-down relative to convention
    # IMPORTANT: UNCOMMENT this line if your IMU is mounted upside down and Z-axis reading is inverted!
    compensated_angular_velocity_z = -angular_velocity_z
    # For now, assuming it's NOT upside down or compensation is handled elsewhere/not needed
    # compensated_angular_velocity_z = angular_velocity_z


    return compensated_angular_velocity_z


# --- Rotation Function ---
def rotate_by_gyro(target_angle_degrees):
    """
    Rotates the robot by a specified angle (in degrees) using IMU feedback.
    Positive angle = Right turn, Negative angle = Left turn.
    Requires IMU to be initialized and calibrated via init_imu().
    Uses proportional control based on remaining angle error.
    """
    global integrated_angle, last_time, imu_initialized

    # Ensure IMU is initialized and calibrated before attempting rotation
    if not imu_initialized or not bias_calibration_done:
        print("Cannot execute rotation: IMU not initialized or calibrated.")
        motor_control.send_command("0 0 0\n") # Ensure motors are stopped if rotate is called when IMU is not ready
        return # Exit function if IMU is not ready

    # Reset integrated angle BEFORE starting THIS rotation
    # This is crucial: each rotation starts from 0 integrated angle for THAT turn
    integrated_angle = 0.0
    last_time = time.time() # Get start time for this rotation

    print(f"Starting rotation: {target_angle_degrees:.2f}�") # Added print with degree symbol

    # --- Rotation Control Parameters (Adjust these for tuning) ---
    # Simple proportional control: Speed = Kp_speed * Error
    Kp_speed = 0.4 # Proportional gain for speed control (Lower = slower, Higher = faster turn)
    angle_tolerance = 1.0 # Stop when within +/- 1.0 degrees of target (increased slightly for stability)
    base_speed = 18 # Max/Base motor speed for rotation (kept at 18)
    min_rotate_speed = 5 # Minimum motor speed to prevent stalling near target (kept at 5)

    # Set a timeout for rotation to prevent infinite loops if IMU fails mid-turn
    rotation_timeout = 10.0 # seconds (Adjust as needed)
    start_rotation_time = time.time()


    # Rotation control loop based on integrated angle
    # Continue as long as error is outside tolerance AND timeout not reached
    while abs(target_angle_degrees - integrated_angle) > angle_tolerance and (time.time() - start_rotation_time) < rotation_timeout:
        current_time = time.time()
        # Calculate time difference since last reading
        # Use a small minimum dt to avoid division by zero issues if loop is very fast
        dt = max(current_time - last_time, 0.0001)
        last_time = current_time # Update last time for the next iteration

        # Read compensated angular velocity (degrees/sec)
        current_velocity = read_imu_data()

        # Integrate velocity over time to get current angle
        integrated_angle += current_velocity * dt

        # Calculate error (remaining angle needed)
        error = target_angle_degrees - integrated_angle

        # Calculate motor speed command based on proportional control
        # The sign of 'error' gives the direction
        rotate_speed_command = Kp_speed * error

        # Apply base speed limit and minimum speed
        # If error is positive, need positive speed (Right turn)
        # If error is negative, need negative speed (Left turn)
        if error > 0: # Need to turn Right (positive angle needed)
             # Speed should be positive, clip between min_rotate_speed and base_speed
             rotate_speed_command = max(min_rotate_speed, min(base_speed, rotate_speed_command))
        else: # Need to turn Left (negative angle needed)
             # Speed should be negative, clip between -base_speed and -min_rotate_speed
             rotate_speed_command = min(-min_rotate_speed, max(-base_speed, rotate_speed_command))


        # Send motor command (Assuming your motor_control takes '0 0 rotate_speed' format)
        # CONVERT TO INTEGER before sending! This caused the float issue.
        send_speed = int(rotate_speed_command)

        # Debug print (Optional, can make log very long)
        # print(f"Time: {current_time - start_rotation_time:.2f}s, Raw: {read_raw_gyro_z()}, Vel: {current_velocity:.2f}, Integ: {integrated_angle:.2f}�, Target: {target_angle_degrees:.2f}�, Error: {error:.2f}�, Speed Cmd: {send_speed}")
        print(f"Integ: {integrated_angle:.2f}� / Target: {target_angle_degrees:.2f}� (Remaining: {error:.2f}�), Speed: {send_speed}")


        # Send motor command
        motor_control.send_command(f"0 0 {send_speed}\n") # Stop motors

        # Check if rotation completed successfully or timed o
