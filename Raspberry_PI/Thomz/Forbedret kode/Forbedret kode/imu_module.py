# Filename: imu_module.py
# Module for interfacing with the IMU (Gyroscope) sensor
# Includes bias calibration, processing, and two rotation functions:
# - rotate_by_gyro: for relative turns based on integrated gyro angle (can drift)
# - rotate_to_heading: for absolute turns using Compass feedback (more accurate for fixed headings)

import smbus    # For I2C communication
import time
import math
from logging_utils import skriv_logg
import motor_control # Import motor_control module
import compass_module # Import compass_module for rotate_to_heading
# Import Heading_Tracker for fusion
from Heading_Tracker import Heading_Tracker # S rg for at denne importen er her


# --- Configuration ---
# IMPORTANT: Verify these settings for YOUR specific IMU chip!
# Common for MPU-6050:
IMU_ADDRESS = 0x68      # I2C address (0x68 or 0x69) - Check with i2cdetect -y 1
PWR_MGMT_1 = 0x6B       # Power Management Register 1
GYRO_ZOUT_H = 0x47      # High byte of Z-axis gyro data

# Gyroscope scale factor (adjust based on your IMU configuration)
# This scales the raw 16-bit value to degrees/second.
# For MPU-6050, depends on GYRO_CONFIG register setting:
# 0x00 (+/- 250 deg/sec) -> 131.0 LSB/deg/sec -> Scale: 1/131.0 = 0.007633
# 0x08 (+/- 500 deg/sec) -> 65.5 LSB/deg/sec  -> Scale: 1/65.5 = 0.015267
# 0x10 (+/- 1000 deg/sec)-> 32.8 LSB/deg/sec  -> Scale: 1/32.8 = 0.03049
# 0x18 (+/- 2000 deg/sec)-> 16.4 LSB/deg/sec  -> Scale: 1/16.4 = 0.06097
# ASSUMING +/- 250 deg/sec range as a common default
GYRO_SCALE_FACTOR = 1.0 / 131.0 # Or adjust based on your GYRO_CONFIG

# --- Global Variables ---
bus = None
integrated_angle = 0.0    # Integrated angle for current rotation (used by rotate_by_gyro)
last_time_gyro = 0.0         # Time of the last IMU reading for integration (for rotate_by_gyro)
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
    global bus, last_time_gyro, gyro_bias_z, bias_calibration_done, imu_initialized
    
    skriv_logg("Initializing IMU...")
    try:
        bus = smbus.SMBus(1) # Use bus 1 on Raspberry Pi - Check your I2C bus number

        # Wake up IMU (command depends on chip, MPU-6050 uses PWR_MGMT_1 = 0)
        # IMPORTANT: Check if this command is correct for your IMU!
        bus.write_byte_data(IMU_ADDRESS, PWR_MGMT_1, 0)
        time.sleep(0.1) # Give sensor time to start

        # Optional: Configure Gyroscope (if needed to set range)
        # For MPU-6050, GYRO_CONFIG is register 0x1B
        # Example: Set to +/- 250 deg/sec (value 0x00)
        bus.write_byte_data(IMU_ADDRESS, 0x1B, 0x00)


# --- Bias Calibration ---
        skriv_logg("Calibrating gyro bias. Keep robot completely still...")
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
        skriv_logg(f"Gyro bias Z calibrated: {gyro_bias_z:.4f}")
        skriv_logg("IMU initialized successfully.") # Success message

        # Set the initial time after calibration for integration
        last_time_gyro = time.time() # Corrected variable name for clarity
        return True # Indicate success
    
    except FileNotFoundError:
        skriv_logg("Error: I2C bus not found. Make sure I2C is enabled in Raspberry Pi configuration.")
        bus = None
        bias_calibration_done = False
        imu_initialized = False
        return False # Indicate failure
    except Exception as e:
        skriv_logg(f"Error during IMU initialization: {e}")
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
        # skriv_logg("Warning: Attempted to read raw gyro before bus initialized.") # Optional debug log
        return 0 # Return 0 if bus is not initialized or error occurred earlier

    try:
        # Read high and low byte for Z-axis gyro
        # IMPORTANT: Verify these register addresses for YOUR IMU! (e.g. 0x47 for MPU-6050)
        gyro_z_h = bus.read_byte_data(IMU_ADDRESS, GYRO_ZOUT_H)
        gyro_z_l = bus.read_byte_data(IMU_ADDRESS, GYRO_ZOUT_H + 1)

        # Combine bytes to a signed 16-bit integer
        raw_value = (gyro_z_h << 8) | gyro_z_l
        if raw_value > 32767: # Handle negative numbers (2's complement)
            raw_value -= 65536

        return raw_value
    
    except Exception:
        # Return 0 on error - this might cause inaccurate integration if reads fail mid-turn
        # skriv_logg(f"Warning: Error reading raw gyro data: {e}") # Optional debug skriv_logg - kan spamme loggen
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
    # IMPORTANT: UNCOMMENT the next line if your IMU is mounted upside down and Z-axis reading is inverted!
    # compensated_angular_velocity_z = -angular_velocity_z
    # Otherwise, use the original angular velocity:
    compensated_angular_velocity_z = angular_velocity_z
    
    # --- Debug logg for IMU Z-akse hastighet - MIDLERTIDIG ---
    # Denne linjen vil kj re HVER gang IMU leses i en rotasjonsloop
    # skriv_logg(f"IMU Debug Z Vel: {compensated_angular_velocity_z:.2f}") # <--- KOMMENTERT UT SOM F R
    # -------------------------------------------------------

    return compensated_angular_velocity_z

# --- Rotation Function (Relative Angle) ---
def rotate_by_gyro(target_angle_degrees):
    """
    Rotates the robot by a specified angle (in degrees) using IMU feedback.
    Positive angle = Right turn, Negative angle = Left turn.
    Requires IMU to be initialized and calibrated via init_imu().
    Uses proportional control based on remaining angle error.
    Best for relative turns (like avoidance maneuvers). Can drift over large angles.
    """
    global integrated_angle, last_time_gyro, imu_initialized, bias_calibration_done # Corrected last_time_gyro

    # Ensure IMU is initialized and calibrated before attempting rotation
    if not imu_initialized or not bias_calibration_done:
        skriv_logg("Cannot execute rotate_by_gyro: IMU not initialized or calibrated.")
        motor_control.send_command("0 0 0\n") # Ensure motors are stopped
        return # Exit function if IMU is not ready
    
    # Reset integrated angle BEFORE starting THIS rotation
    # This is crucial: each relative rotation starts from 0 integrated angle for THAT turn
    integrated_angle = 0.0
    last_time_gyro = time.time() # Get start time for this rotation (Corrected variable name)

    skriv_logg(f"Starting rotation (gyro): {target_angle_degrees:.2f} ") # Added skriv_logg with degree symbol

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
    # The condition checks if the *absolute* difference is greater than tolerance
    while abs(target_angle_degrees - integrated_angle) > angle_tolerance and (time.time() - start_rotation_time) < rotation_timeout:
        current_time = time.time()
        # Calculate time difference since last reading
        # Use a small minimum dt to avoid division by zero issues if loop is very fast
        dt = max(current_time - last_time_gyro, 0.0001) # Corrected variable name
        last_time_gyro = current_time # Update last time for the next iteration (Corrected variable name)

        # Read compensated angular velocity (degrees/sec)
        current_velocity = read_imu_data()

        # Integrate velocity over time to get current angle
        integrated_angle += current_velocity * dt

        # Calculate error (remaining angle needed)
        # Filename: imu_module.py
# Module for interfacing with the IMU (Gyroscope) sensor
# Includes bias calibration, processing, and two rotation functions:
# - rotate_by_gyro: for relative turns based on integrated gyro angle (can drift)
# - rotate_to_heading: for absolute turns using Compass feedback (more accurate for fixed headings)

import smbus    # For I2C communication
import time
import math
from logging_utils import skriv_logg
import motor_control # Import motor_control module
import compass_module # Import compass_module for rotate_to_heading
# Import Heading_Tracker for fusion
from Heading_Tracker import Heading_Tracker # S rg for at denne importen er her


# --- Configuration ---
# IMPORTANT: Verify these settings for YOUR specific IMU chip!
# Common for MPU-6050:
IMU_ADDRESS = 0x68      # I2C address (0x68 or 0x69) - Check with i2cdetect -y 1
PWR_MGMT_1 = 0x6B       # Power Management Register 1
GYRO_ZOUT_H = 0x47      # High byte of Z-axis gyro data

# Gyroscope scale factor (adjust based on your IMU configuration)
# This scales the raw 16-bit value to degrees/second.
# For MPU-6050, depends on GYRO_CONFIG register setting:
# 0x00 (+/- 250 deg/sec) -> 131.0 LSB/deg/sec -> Scale: 1/131.0 = 0.007633
# 0x08 (+/- 500 deg/sec) -> 65.5 LSB/deg/sec  -> Scale: 1/65.5 = 0.015267
# 0x10 (+/- 1000 deg/sec)-> 32.8 LSB/deg/sec  -> Scale: 1/32.8 = 0.03049
# 0x18 (+/- 2000 deg/sec)-> 16.4 LSB/deg/sec  -> Scale: 1/16.4 = 0.06097
# ASSUMING +/- 250 deg/sec range as a common default
GYRO_SCALE_FACTOR = 1.0 / 131.0 # Or adjust based on your GYRO_CONFIG

# --- Global Variables ---
bus = None
integrated_angle = 0.0    # Integrated angle for current rotation (used by rotate_by_gyro)
last_time_gyro = 0.0         # Time of the last IMU reading for integration (for rotate_by_gyro)
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
    global bus, last_time_gyro, gyro_bias_z, bias_calibration_done, imu_initialized
    
    skriv_logg("Initializing IMU...")
    try:
        bus = smbus.SMBus(1) # Use bus 1 on Raspberry Pi - Check your I2C bus number

        # Wake up IMU (command depends on chip, MPU-6050 uses PWR_MGMT_1 = 0)
        # IMPORTANT: Check if this command is correct for your IMU!
        bus.write_byte_data(IMU_ADDRESS, PWR_MGMT_1, 0)
        time.sleep(0.1) # Give sensor time to start

        # Optional: Configure Gyroscope (if needed to set range)
        # For MPU-6050, GYRO_CONFIG is register 0x1B
        # Example: Set to +/- 250 deg/sec (value 0x00)
        bus.write_byte_data(IMU_ADDRESS, 0x1B, 0x00)


# --- Bias Calibration ---
        skriv_logg("Calibrating gyro bias. Keep robot completely still...")
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
        skriv_logg(f"Gyro bias Z calibrated: {gyro_bias_z:.4f}")
        skriv_logg("IMU initialized successfully.") # Success message

        # Set the initial time after calibration for integration
        last_time_gyro = time.time() # Corrected variable name for clarity
        return True # Indicate success
    
    except FileNotFoundError:
        skriv_logg("Error: I2C bus not found. Make sure I2C is enabled in Raspberry Pi configuration.")
        bus = None
        bias_calibration_done = False
        imu_initialized = False
        return False # Indicate failure
    except Exception as e:
        skriv_logg(f"Error during IMU initialization: {e}")
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
        # skriv_logg("Warning: Attempted to read raw gyro before bus initialized.") # Optional debug log
        return 0 # Return 0 if bus is not initialized or error occurred earlier

    try:
        # Read high and low byte for Z-axis gyro
        # IMPORTANT: Verify these register addresses for YOUR IMU! (e.g. 0x47 for MPU-6050)
        gyro_z_h = bus.read_byte_data(IMU_ADDRESS, GYRO_ZOUT_H)
        gyro_z_l = bus.read_byte_data(IMU_ADDRESS, GYRO_ZOUT_H + 1)

        # Combine bytes to a signed 16-bit integer
        raw_value = (gyro_z_h << 8) | gyro_z_l
        if raw_value > 32767: # Handle negative numbers (2's complement)
            raw_value -= 65536

        return raw_value
    
    except Exception:
        # Return 0 on error - this might cause inaccurate integration if reads fail mid-turn
        # skriv_logg(f"Warning: Error reading raw gyro data: {e}") # Optional debug skriv_logg - kan spamme loggen
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
    # IMPORTANT: UNCOMMENT the next line if your IMU is mounted upside down and Z-axis reading is inverted!
    # compensated_angular_velocity_z = -angular_velocity_z
    # Otherwise, use the original angular velocity:
    compensated_angular_velocity_z = angular_velocity_z
    
    # --- Debug logg for IMU Z-akse hastighet - MIDLERTIDIG ---
    # Denne linjen vil kj re HVER gang IMU leses i en rotasjonsloop
    # skriv_logg(f"IMU Debug Z Vel: {compensated_angular_velocity_z:.2f}") # <--- KOMMENTERT UT SOM F R
    # -------------------------------------------------------

    return compensated_angular_velocity_z

# --- Rotation Function (Relative Angle) ---
def rotate_by_gyro(target_angle_degrees):
    """
    Rotates the robot by a specified angle (in degrees) using IMU feedback.
    Positive angle = Right turn, Negative angle = Left turn.
    Requires IMU to be initialized and calibrated via init_imu().
    Uses proportional control based on remaining angle error.
    Best for relative turns (like avoidance maneuvers). Can drift over large angles.
    """
    global integrated_angle, last_time_gyro, imu_initialized, bias_calibration_done # Corrected last_time_gyro

    # Ensure IMU is initialized and calibrated before attempting rotation
    if not imu_initialized or not bias_calibration_done:
        skriv_logg("Cannot execute rotate_by_gyro: IMU not initialized or calibrated.")
        motor_control.send_command("0 0 0\n") # Ensure motors are stopped
        return # Exit function if IMU is not ready
    
    # Reset integrated angle BEFORE starting THIS rotation
    # This is crucial: each relative rotation starts from 0 integrated angle for THAT turn
    integrated_angle = 0.0
    last_time_gyro = time.time() # Get start time for this rotation (Corrected variable name)

    skriv_logg(f"Starting rotation (gyro): {target_angle_degrees:.2f} ") # Added skriv_logg with degree symbol

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
    # The condition checks if the *absolute* difference is greater than tolerance
    while abs(target_angle_degrees - integrated_angle) > angle_tolerance and (time.time() - start_rotation_time) < rotation_timeout:
        current_time = time.time()
        # Calculate time difference since last reading
        # Use a small minimum dt to avoid division by zero issues if loop is very fast
        dt = max(current_time - last_time_gyro, 0.0001) # Corrected variable name
        last_time_gyro = current_time # Update last time for the next iteration (Corrected variable name)

        # Read compensated angular velocity (degrees/sec)
        current_velocity = read_imu_data()

        # Integrate velocity over time to get current angle
        integrated_angle += current_velocity * dt

        # Calculate error (remaining angle needed)
        # Filename: imu_module.py
# Module for interfacing with the IMU (Gyroscope) sensor
# Includes bias calibration, processing, and two rotation functions:
# - rotate_by_gyro: for relative turns based on integrated gyro angle (can drift)
# - rotate_to_heading: for absolute turns using Compass feedback (more accurate for fixed headings)

import smbus    # For I2C communication
import time
import math
from logging_utils import skriv_logg
import motor_control # Import motor_control module
import compass_module # Import compass_module for rotate_to_heading
# Import Heading_Tracker for fusion
from Heading_Tracker import Heading_Tracker # S rg for at denne importen er her


# --- Configuration ---
# IMPORTANT: Verify these settings for YOUR specific IMU chip!
# Common for MPU-6050:
IMU_ADDRESS = 0x68      # I2C address (0x68 or 0x69) - Check with i2cdetect -y 1
PWR_MGMT_1 = 0x6B       # Power Management Register 1
GYRO_ZOUT_H = 0x47      # High byte of Z-axis gyro data

# Gyroscope scale