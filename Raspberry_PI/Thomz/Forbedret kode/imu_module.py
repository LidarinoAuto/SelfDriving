# -*- coding: utf-8 -*-
# Filename: imu_module.py
# Module for interfacing with the IMU (Gyroscope) sensor
# Includes bias calibration and upside-down compensation

import smbus # For I2C communication
import time
import math # Can be useful, but maybe not needed for simple bias/integration

# --- Configuration ---
# I2C address for the MPU-6050 or compatible IMU (common addresses)
IMU_ADDRESS = 0x68 # Check your specific IMU breakout board, 0x69 is also common
# Or use the address for QMI8658/QMI8610 if that's what you have
# QMI8658_ADDRESS = 0x6B # Example address if not MPU-6050

# Register addresses (these depend on the specific IMU model, e.g., MPU-6050 registers)
# If you have QMI8658 or similar, these registers will be different!
# You need to find the register map for your specific IMU chip.
# For MPU-6050:
PWR_MGMT_1 = 0x6B
GYRO_XOUT_H = 0x43 # High byte of X-axis gyro data
GYRO_YOUT_H = 0x45 # High byte of Y-axis gyro data
GYRO_ZOUT_H = 0x47 # High byte of Z-axis gyro data
# Assuming you are using the Z-axis for yaw (rotation on horizontal plane)

# Gyroscope scale factor (adjust based on your IMU configuration)
# Common settings: +/- 250, 500, 1000, 2000 degrees/second
# If set to +/- 250 deg/sec, raw reading of 32768 = 250 deg/sec
# Scale factor = 250.0 / 32768.0
GYRO_SCALE_FACTOR = 250.0 / 32768.0 # Example for +/- 250 deg/sec

# --- Global Variables ---
bus = None
integrated_angle = 0.0 # Integrated angle in degrees
last_time = 0.0      # Time of the last IMU reading
gyro_bias_z = 0.0    # Global variable to store the calculated Z-axis gyro bias
bias_calibration_done = False # Flag to check if calibration is complete


# --- Initialization ---
def init_imu():
    """
    Initializes the IMU sensor and performs bias calibration.
    Requires the robot to be stationary during calibration.
    """
    global bus, last_time, gyro_bias_z, bias_calibration_done

    print("Initializing IMU...")
    # Initialize I2C bus (usually bus 1 on Raspberry Pi)
    try:
        bus = smbus.SMBus(1) # Use bus 1 on Raspberry Pi
        # Wake up the MPU-6050 (PWR_MGMT_1 register) if it's MPU-6050
        # If using QMI8658 or similar, this register/command might be different!
        bus.write_byte_data(IMU_ADDRESS, PWR_MGMT_1, 0) # Assuming PWR_MGMT_1 and address 0x68 for MPU-6050
        time.sleep(0.1) # Wait for sensor to start up

        # --- Bias Calibration ---
        print("Calibrating gyro bias. Keep the robot COMPLETELY STILL...")
        calibration_readings = 200 # Number of readings for calibration
        total_gyro_z = 0.0

        # Read raw gyro data multiple times while stationary
        for i in range(calibration_readings):
            raw_z = read_raw_gyro_z() # Use a helper to get raw Z data
            total_gyro_z += raw_z
            time.sleep(0.01) # Small delay between readings

        # Calculate average bias
        gyro_bias_z = total_gyro_z / calibration_readings
        bias_calibration_done = True # Set flag
        print(f"Gyro bias Z calibrated: {gyro_bias_z:.4f}")
        print("IMU initialized.") # This print was originally here, now after calibration

        # Set the initial time after calibration
        last_time = time.time()

    except FileNotFoundError:
        print("Error: I2C bus not found. Make sure I2C is enabled in Raspberry Pi configuration.")
        bus = None # Ensure bus is None
        bias_calibration_done = False # Calibration failed
    except Exception as e:
        print(f"Error during IMU initialization: {e}")
        # This catch block will print the Remote I/O error
        bus = None # Ensure bus is None
        bias_calibration_done = False # Calibration failed


# Helper function to read raw Z-axis gyro data
# Adjust register addresses and byte reading logic based on your IMU chip
def read_raw_gyro_z():
    """Reads raw Z-axis gyroscope data from the IMU."""
    if bus is None: # or not bias_calibration_done: # Can read raw even before calibration? Yes.
        # print("IMU bus not initialized.") # Avoid spamming if init failed
        return 0 # Return 0 if bus is not ready

    try:
        # Read high and low byte for Z-axis gyro
        # Assumes GYRO_ZOUT_H and GYRO_ZOUT_H+1 are the correct registers for your IMU
        # If using QMI8658/QMI8610, registers are different (e.g., 0x04, 0x05 for gyro Z)
        gyro_z_h = bus.read_byte_data(IMU_ADDRESS, GYRO_ZOUT_H)
        gyro_z_l = bus.read_byte_data(IMU_ADDRESS, GYRO_ZOUT_H + 1)

        # Combine bytes to a signed 16-bit integer
        raw_value = (gyro_z_h << 8) | gyro_z_l
        if raw_value > 32767: # Check for negative number (2's complement)
            raw_value -= 65536

        return raw_value

    except Exception as e:
        # print(f"Error reading raw gyro data: {e}") # Avoid spamming if read fails repeatedly
        # Return 0 on error - this might cause inaccurate integration if reads fail mid-turn
        return 0


# --- Data Reading and Processing ---
# This function reads processed IMU data (Z-axis angular velocity in degrees/sec)
# after bias correction and scaling.
def read_imu_data():
    """
    Reads processed IMU data (Z-axis angular velocity in degrees/sec)
    after bias correction and scaling.
    Returns 0.0 if IMU is not initialized or calibrated.
    """
    global last_time, gyro_bias_z, bias_calibration_done

    # Only process if IMU is initialized and calibrated
    if bus is None or not bias_calibration_done:
        # print("IMU not initialized or not calibrated for reading.") # Avoid spamming
        return 0.0 # Return 0 if IMU is not ready/calibrated


    # Read raw Z-axis gyro data
    raw_gyro_z = read_raw_gyro_z() # Call the helper function

    # --- Apply Bias Correction ---
    # Subtract the calculated bias from the raw reading
    corrected_gyro_z = raw_gyro_z - gyro_bias_z

    # --- Apply Scale Factor ---
    # Convert raw value to degrees per second
    angular_velocity_z = corrected_gyro_z * GYRO_SCALE_FACTOR

    # --- Apply Upside-Down Compensation ---
    # Based on your previous logs showing 'comp: -Raw gyro'
    # If Z-axis rotation is inverted when mounted upside down, negate it.
    # Adjust this if your mounting or IMU orientation is different.
    compensated_angular_velocity_z = -angular_velocity_z # Negate for upside-down mounting

    # Return the compensated angular velocity in degrees per second
    return compensated_angular_velocity_z


# --- Rotation Function ---
# This function uses the IMU to perform a rotation by integrating angular velocity.
# You had this function in your original code. Ensure this version is present and correct.
def rotate_by_gyro(target_angle_degrees):
    """
    Rotates the robot by a specified angle (in degrees) using IMU feedback.
    Positive angle = Right turn, Negative angle = Left turn.
    Requires IMU to be initialized and calibrated.
    """
    global integrated_angle, last_time
    # Reset integrated angle before starting THIS rotation
    integrated_angle = 0.0
    last_time = time.time() # Get start time for this rotation

    if bus is None or not bias_calibration_done:
        print("Cannot execute rotation: IMU not initialized or calibrated.")
        return # Exit function if IMU is not ready

    print(f"Starting rotation: {target_angle_degrees:.2f}�") # Added print

    # --- PID Controller Parameters (These need tuning for best performance) ---
    Kp = 0.8 # Proportional gain (Start with this, adjust)
    Ki = 0.0 # Integral gain (Often not needed for simple turns, can cause overshoot)
    Kd = 0.05 # Derivative gain (Can help dampen oscillation, start small)
    integral_error = 0.0 # Initialize integral
    last_error = 0.0 # Initialize last error

    # --- Rotation Control Loop ---
    # Use a small tolerance for reaching the target angle
    angle_tolerance = 0.5 # Stop when within +/- 0.5 degrees of target

    # Determine motor command direction based on target angle
    base_speed = 18 # Base motor speed for rotation (adjust as needed)
    turn_dir = 1 if target_angle_degrees > 0 else -1 # 1 for right, -1 for left

    # Send initial motor command to start rotating
    # Rotate command is typically (0, Left_Speed, Right_Speed) where speeds are opposite
    # Assuming positive is forward for Left/Right speed
    # For rotation: left_speed = -turn_dir * speed, right_speed = turn_dir * speed
    # Or use the format (Forward, Side, Rotate) from motor_control if it supports that
    # Assuming your motor_control.send_command(f"0 0 {rotate_speed}\n") handles this
    # Let's send rotate speed directly:
    motor_control.send_command(f"0 0 {turn_dir * base_speed}\n") # Send initial rotation command


    # Loop while we are outside the target tolerance
    # Check remaining angle vs target angle, considering direction
    while abs(target_angle_degrees - integrated_angle) > angle_tolerance:
        current_time = time.time()
        # dt = current_time - last_time # Moved dt calculation here
        # Avoid zero dt if loop is too fast
        dt = max(current_time - last_time, 0.0001) # Minimum dt to avoid division by zero issues if needed
        last_time = current_time # Update last time for the next iteration

        # Read compensated angular velocity (degrees/sec)
        current_velocity = read_imu_data() # *** CALLS THE CORRECTED READING FUNCTION ***

        # Integrate velocity to get current angle
        integrated_angle += current_velocity * dt

        # --- PID Calculation ---
        error = target_angle_degrees - integrated_angle
        integral_error += error * dt
        derivative_error = (error - last_error) / dt
        last_error = error

        # PID output is the control signal (e.g., desired angular velocity or motor speed adjustment)
        # For rotation, PID output usually controls the *speed* of rotation
        # Let's map PID output directly to rotation speed adjustment for simplicity initially
        # You might need more complex mapp     :E# -*- coding: utf-8 -*-
# Filename: imu_module.py
# Module for interfacing with the IMU (Gyroscope) sensor
# Includes bias calibration and upside-down compensation

import smbus # For I2C communication
import time
import math # Can be useful, but maybe not needed for simple bias/integration

# --- Configuration ---
# I2C address for the MPU-6050 or compatible IMU (common addresses)
IMU_ADDRESS = 0x68 # Check your specific IMU breakout board, 0x69 is also common
# Or use the address for QMI8658/QMI8610 if that's what you have
# QMI8658_ADDRESS = 0x6B # Example address if not MPU-6050

# Register addresses (these depend on the specific IMU model, e.g., MPU-6050 registers)
# If you have QMI8658 or similar, these registers will be different!
# You need to find the register map for your specific IMU chip.
# For MPU-6050:
PWR_MGMT_1 = 0x6B
GYRO_XOUT_H = 0x43 # High byte of X-axis gyro data
GYRO_YOUT_H = 0x45 # High byte of Y-axis gyro data
GYRO_ZOUT_H = 0x47 # High byte of Z-axis gyro data
# Assuming you are using the Z-axis for yaw (rotation on horizontal plane)

# Gyroscope scale factor (adjust based on your IMU configuration)
# Common settings: +/- 250, 500, 1000, 2000 degrees/second
# If set to +/- 250 deg/sec, raw reading of 32768 = 250 deg/sec
# Scale factor = 250.0 / 32768.0
GYRO_SCALE_FACTOR = 250.0 / 32768.0 # Example for +/- 250 deg/sec

# --- Global Variables ---
bus = None
integrated_angle = 0.0 # Integrated angle in degrees
last_time = 0.0      # Time of the last IMU reading
gyro_bias_z = 0.0    # Global variable to store the calculated Z-axis gyro bias
bias_calibration_done = False # Flag to check if calibration is complete


# --- Initialization ---
def init_imu():
    """
    Initializes the IMU sensor and performs bias calibration.
    Requires the robot to be stationary during calibration.
    """
    global bus, last_time, gyro_bias_z, bias_calibration_done

    print("Initializing IMU...")
    # Initialize I2C bus (usually bus 1 on Raspberry Pi)
    try:
        bus = smbus.SMBus(1) # Use bus 1 on Raspberry Pi
        # Wake up the MPU-6050 (PWR_MGMT_1 register) if it's MPU-6050
        # If using QMI8658 or similar, this register/command might be different!
        bus.write_byte_data(IMU_ADDRESS, PWR_MGMT_1, 0) # Assuming PWR_MGMT_1 and address 0x68 for MPU-6050
        time.sleep(0.1) # Wait for sensor to start up

        # --- Bias Calibration ---
        print("Calibrating gyro bias. Keep the robot COMPLETELY STILL...")
        calibration_readings = 200 # Number of readings for calibration
        total_gyro_z = 0.0

        # Read raw gyro data multiple times while stationary
        for i in range(calibration_readings):
            raw_z = read_raw_gyro_z() # Use a helper to get raw Z data
            total_gyro_z += raw_z
            time.sleep(0.01) # Small delay between readings

        # Calculate average bias
        gyro_bias_z = total_gyro_z / calibration_readings
        bias_calibration_done = True # Set flag
        print(f"Gyro bias Z calibrated: {gyro_bias_z:.4f}")
        print("IMU initialized.") # This print was originally here, now after calibration

        # Set the initial time after calibration
        last_time = time.time()

    except FileNotFoundError:
        print("Error: I2C bus not found. Make sure I2C is enabled in Raspberry Pi configuration.")
        bus = None # Ensure bus is None
        bias_calibration_done = False # Calibration failed
    except Exception as e:
        print(f"Error during IMU initialization: {e}")
        # This catch block will print the Remote I/O error
        bus = None # Ensure bus is None
        bias_calibration_done = False # Calibration failed


# Helper function to read raw Z-axis gyro data
# Adjust register addresses and byte reading logic based on your IMU chip
def read_raw_gyro_z():
    """Reads raw Z-axis gyroscope data from the IMU."""
    if bus is None: # or not bias_calibration_done: # Can read raw even before calibration? Yes.
        # print("IMU bus not initialized.") # Avoid spamming if init failed
        return 0 # Return 0 if bus is not ready

    try:
        # Read high and low byte for Z-axis gyro
        # Assumes GYRO_ZOUT_H and GYRO_ZOUT_H+1 are the correct registers for your IMU
        # If using QMI8658/QMI8610, registers are different (e.g., 0x04, 0x05 for gyro Z)
        gyro_z_h = bus.read_byte_data(IMU_ADDRESS, GYRO_ZOUT_H)
        gyro_z_l = bus.read_byte_data(IMU_ADDRESS, GYRO_ZOUT_H + 1)

        # Combine bytes to a signed 16-bit integer
        raw_value = (gyro_z_h << 8