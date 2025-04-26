# -*- coding: utf-8 -*-
# Filename: imu_module.py
# Module for interfacing with the IMU (Gyroscope) sensor
# Includes bias calibration and basic proportional rotation control.

import smbus      # For I2C communication
import time
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


# --- Initialization ---
def init_imu():
    """
    Initializes the IMU sensor (I2C) and performs gyro bias calibration.
    Requires the robot to be stationary during calibration.
    Returns True if initialization and calibration succeed, False otherwise.
    """
    global bus, last_time, gyro_bias_z, bias_calibration_done

    print("Initializing IMU...")
    try:
        bus = smbus.SMBus(1) # Use bus 1 on Raspberry Pi

        # Wake up IMU (command depends on chip, MPU-6050 uses PWR_MGMT_1 = 0)
        # IMPORTANT: Check if this command is correct for your IMU!
        bus.write_byte_data(IMU_ADDRESS, PWR_MGMT_1, 0)
        time.sleep(0.1) # Give sensor time to start

        # --- Bias Calibration ---
        print("Calibrating gyro bias. Keep the robot COMPLETELY STILL...")
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
        print(f"Gyro bias Z calibrated: {gyro_bias_z:.4f}")
        print("IMU initialized.") # Success message

        # Set the initial time after calibration for integration
        last_time = time.time()
        return True # Indicate success

    except FileNotFoundError:
        print("Error: I2C bus not found. Make sure I2C is enabled in Raspberry Pi configuration.")
        bus = None
        bias_calibration_done = False
        return False # Indicate failure
    except Exception as e:
        print(f"Error during IMU initialization: {e}")
        bus = None
        bias_calibration_done = False
        return False # Indicate failure


# Helper function to read raw Z-axis gyro data
# IMPORTANT: Adjust register addresses and byte logic for YOUR specific IMU chip!
def read_raw_gyro_z():
    """Reads raw Z-axis gyroscope data from the IMU."""
    global bus
    if bus is None:
        return 0 # Return 0 if bus is not initialized

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
        return 0


# --- Data Reading and Processing ---
def read_imu_data():
    """
    Reads processed IMU data (Z-axis angular velocity in degrees/sec)
    after bias correction, scaling, and orientation compensation.
    Returns 0.0 if IMU is not initialized or calibrated.
    """
    global gyro_bias_z, bias_calibration_done

    # Only process if IMU is initialized and calibrated
    if bus is None or not bias_calibration_done:
        return 0.0

    raw_gyro_z = read_raw_gyro_z() # Read raw data

    # Apply Bias Correction
    corrected_gyro_z = raw_gyro_z - gyro_bias_z

    # Apply Scale Factor (convert raw to deg/sec)
    angular_velocity_z = corrected_gyro_z * GYRO_SCALE_FACTOR

    # Apply Upside-Down Compensation
    # Negate Z-axis velocity if IMU is mounted upside-down relative to convention
    compensated_angular_velocity_z = -angular_velocity_z

    return compensated_angular_velocity_z


# --- Rotation Function ---
def rotate_by_gyro(target_angle_degrees):
    """
    Rotates the robot by a specified angle (in degrees) using IMU feedback.
    Positive angle = Right turn, Negative angle = Left turn.
    Requires IMU to be initialized and calibrated via init_imu().
    Uses proportional control based on remaining angle error.
    """
    global integrated_angle, last_time
    # Reset integrated angle before starting THIS rotation
    integrated_angle = 0.0
    last_time = time.time() # Get start time for this rotation

    if bus is None or not bias_calibration_done:
        print("Cannot execute rotation: IMU not initialized or calibrated.")
        return # Exit function if IMU is not ready

    print(f"Starting rotation: {target_angle_degrees:.2f}�")

    # --- Rotation Control Parameters (Adjust these for tuning) ---
    # Simple proportional control: Speed = Kp_speed * Error
    Kp_speed = 0.4 # Proportional gain for speed control (Lower = slower, Higher = faster turn)
    angle_tolerance = 0.5 # Stop when within +/- 0.5 degrees of target
    base_speed = 18 # Max/Base motor speed for rotation
    min_rotate_speed = 5 # Minimum motor speed to prevent stalling near target


    # Rotation control loop based on integrated angle
    while abs(target_angle_degrees - integrated_angle) > angle_tolerance:
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
        rotate_speed_command = Kp_speed * error

        # Apply sign and clip the speed command
        # If error is positive, need positive speed (Right turn)
        # If error is negative, need negative speed (Left turn)
        if error > 0: # Need to turn Right
             # Speed should be positive, between min_rotate_speed and max_rotate_speed
             rotate_speed_command = max(min_rotate_speed, min(base_speed, rotate_speed_command))
        else: # Need to turn Left
             # Speed should be negative, between -base_speed and -min_rotate_speed
             rotate_speed_command = min(-min_rotate_speed, max(-base_speed, rotate_speed_command))

        # Send motor command (Assuming your motor_control takes '0 0 rotate_speed' format)
        # Example Debug print:
        # print(f"Raw: {read_raw_gyro_z():.2f}, Corrected: {read_imu_data():.2f}, Integ: {integrated_angle:.2f}�, Err: {error:.2f}�, Speed: {int(rotate_speed_command)}")
        print(f"Integ: {integrated_angle:.2f}� / Target: {target_angle_degrees:.2f}� (Remaining: {error:.2f}�)")


        # Send motor command
        motor_control.send_command(f"0 0 {int(rotate_speed_command)}\n")

        # Small delay for control loop frequency
        time.sleep(0.01) # Aim for ~100 Hz control loop

    # After the loop finishes (target angle reached within tolerance), stop motors
    motor_control.send_command("0 0 0\n")
    print(f"Rotation completed. Reached {integrated_angle:.2f}� of target {target_angle_degrees:.2f}�.")


# --- Cleanup ---
def cleanup():
    """
    Clean up resources used by the IMU module.
    Closes the I2C bus if it was opened.
    """
    print("IMU cleanup: Closing I2C bus...")
    global bus
    if bus is not None:
        try:
            bus.close()
            print("IMU cleanup: I2C bus closed.")
        except Exception as e:
            print(f"IMU cleanup: Error closing I2C bus: {e}")
        bus = None # Clear the bus reference

# --- Example usage (if this module is run directly for testing) ---
# This block is commented out by default. Uncomment for isolation testing.
# if __name__ == "__main__":
#     print("Running IMU module isolation test...")
#     # Note: This test requires a physical IMU connected via I2C
#     # and requires a dummy/mock motor_control module to be available
#     # if you uncomment the rotate_by_gyro test.

#     # Example of initializing and reading data:
#     # if init_imu(): # Initialize and calibrate
#     #     print("IMU ready and calibrated.")
#     #     print("Reading IMU data for 5 seconds...")
#     #     start_test_time = time.time()
#     #     while (time.time() - start_test_time) < 5:
#     #         velocity = read_imu_data()
#     #         print(f"Time: {time.time() - start_test_time:.2f}s, Velocity Z: {velocity:.2f} deg/sec")
#     #         time.sleep(0.05) # Read frequency example
#     # else:
#     #      print("IMU initialization or calibration failed. Cannot run test.")

#     # Example of calling cleanup manually after test
#     # cleanup()