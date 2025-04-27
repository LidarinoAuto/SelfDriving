# -*- coding: utf-8 -*-
# Filename: main.py
# Main program for robot navigation with sensors and motor control
# Orchestrates sensor inputs and motor control for obstacle avoidance.

import time
# Removed math import as it's not explicitly used in this simplified version
# Removed threading import as it's handled within modules like lidar_module
import sys # Import sys if needed elsewhere, but not strictly needed for this structure


# Import modules for robot components
import motor_control
import ultrasound_module
import imu_module
import compass_module
import lidar_module


# --- Configuration Constants ---
LIDAR_STOP_DISTANCE_CM = 27.0  # Distance Lidar should stop at (cm)
US_STOP_DISTANCE_CM = 8.0    # Distance Ultrasound should stop at (cm)
IMU_NORTH_HEADING = 0.0      # Compass heading for North (adjust if needed)
FALLBACK_TURN_ANGLE = 25.0   # Angle for fallback turns (degrees)
US_AVOID_TURN_ANGLE = 45.0   # Angle for US-triggered turns (degrees)


# --- Global State Variables ---
# Flags to track if sensors were successfully initialized
# Used to conditionally use sensors/modules
imu_initialized = False
compass_initialized = False
lidar_initialized = False
ultrasound_initialized = False # Added flag for ultrasound initialization


# State variable for alternating fallback turn direction (+1 for right, -1 for left)
last_fallback_turn_direction = 1


# --- Sensor Reading and Obstacle Detection ---
def check_for_obstacles():
    """
    Reads sensor data and determines if an obstacle is detected.
    Returns obstacle status and relevant sensor data.
    """
    global lidar_initialized, ultrasound_initialized # Need to check if sensors were initialized

    # --- Read Sensor Data ---
    # Lidar reading (default to infinity if not initialized or no valid data)
    current_lidar_distance = lidar_module.get_median_lidar_reading_cm() if lidar_initialized else float('inf')

    # Ultrasound reading (using the function from your likely ultrasound_module.py)
    # Assumes get_triggered_ultrasound_info returns a tuple: (list of triggered sensor IDs, dictionary of distances {id: distance})
    # Pass the threshold to the function
    # Handle case where ultrasound might not be initialized
    triggered_us_sensors = [] # Default empty list
    current_us_distances = {} # Default empty dictionary
    ultrasound_obstacle_status = False

    if ultrasound_initialized: # Only call US function if initialized
        try:
            # Assuming this function exists and returns (list of triggered IDs, dict of distances)
            us_info = ultrasound_module.get_triggered_ultrasound_info(US_STOP_DISTANCE_CM)
            triggered_us_sensors = us_info[0] # List of triggered sensor IDs (e.g., [0, 1])
            current_us_distances = us_info[1] # Dictionary of distances {id: distance} (e.g., {0: 5.5, 1: 7.0})
            ultrasound_obstacle_status = len(triggered_us_sensors) > 0 # Obstacle if any sensor triggered
        except Exception as e:
            print(f"Error reading ultrasound data: {e}")
            # Keep obstacle_status as False, or set to True if error should mean stop


    # --- Determine Obstacle Status ---
    # Lidar obstacle if reading is valid and below threshold (only if Lidar initialized)
    lidar_obstacle_status = (current_lidar_distance != float('inf') and current_lidar_distance < LIDAR_STOP_DISTANCE_CM) if lidar_initialized else False


    # Obstacle detected if EITHER Lidar OR Ultrasound detected one
    obstacle_detected = lidar_obstacle_status or ultrasound_obstacle_status

    # Return combined status and sensor data
    # Return US distances dict and triggered list for potential use in avoidance printing
    return obstacle_detected, lidar_obstacle_status, ultrasound_obstacle_status, current_lidar_distance, triggered_us_sensors, current_us_distances


# --- Avoidance Maneuver ---
# Updated to use the US data format from get_triggered_ultrasound_info
def perform_avoidance(lidar_triggered, us_triggered_list, us_distances_dict):
    """
    Performs an avoidance maneuver based on which sensor(s) triggered detection.
    Requires IMU to be initialized for rotation.
    us_triggered_list is a list of sensor IDs (e.g., [0, 1]), us_distances_dict is {id: distance}.
    """
    global imu_initialized, last_fallback_turn_direction, FALLBACK_TURN_ANGLE, US_AVOID_TURN_ANGLE

    print("Obstacle detected! Stopping...")
    motor_control.send_command("0 0 0\n") # Stop robot immediately

    # --- Determine Turn Angle Based on Trigger ---
    turn_angle = 0 # Default: no turn

    # Check if specific US sensors triggered individually (using IDs from the list)
    # Assuming IDs 0 and 1 are front sensors relevant for turning away
    is_us_0_triggered = 0 in us_triggered_list # Likely Physical Right Front
    is_us_1_triggered = 1 in us_triggered_list # Likely Physical Left Front
    is_back_triggered = any(us_id in us_triggered_list for us_id in [2, 3]) # Assuming IDs 2, 3 are back sensors


    # Prioritize US-specific turns if applicable and only one front US triggered
    # This logic can be refined based on robot design and desired avoidance
    if is_us_0_triggered and not is_us_1_triggered and not is_back_triggered: # Only US0 (Right Front) triggered
         print(f"US triggered (Right Front): {us_distances_dict.get(0, float('inf')):.2f}cm. Turning Left (-{US_AVOID_TURN_ANGLE:.0f}�)...")
         turn_angle = -US_AVOID_TURN_ANGLE # Turn Left
    elif is_us_1_triggered and not is_us_0_triggered and not is_back_triggered: # Only US1 (Left Front) triggered
         print(f"US triggered (Left Front): {us_distances_dict.get(1, float('inf')):.2f}cm. Turning Right (+{US_AVOID_TURN_ANGLE:.0f}�)...")
         turn_angle = US_AVOID_TURN_ANGLE # Turn Right
    # If both front US triggered OR Lidar triggered OR back US triggered -> use fallback
    elif lidar_triggered or is_back_triggered or (is_us_0_triggered and is_us_1_triggered):
        if lidar_triggered:
            print(f"LiDAR triggered. Turning...") # Lidar distance already printed in main loop
        elif len(us_triggered_list) > 0: # Any US triggered (including back or both front)
             # Print details of triggered US sensors in this case
             details = ", ".join([f"US {i}: {us_distances_dict.get(i, float('inf')):.2f}cm" for i in us_triggered_list])
             print(f"US triggered ({details}). Turning...")


        # Fallback maneuver: alternate left/right turns
        turn_angle = FALLBACK_TURN_ANGLE * last_fallback_turn_direction
        print(f"Fallback turn: {turn_angle:.0f}�")
        # Switch direction for next fallback
        last_fallback_turn_direction *= -1

    else:
        # This case should not be reached if perform_avoidance is only called when obstacle_detected is True
        print("Obstacle detected, but no specific avoidance rule matched. Stopping.")
        motor_control.send_command("0 0 0\n")
        time.sleep(1.0) # Pause
        return # Exit avoidance without turn

    # --- Execute the Turn ---
    if turn_angle != 0: # Only attempt turn if an angle was determined
        print(f"Executing rotation of {turn_angle:.0f}�...")
        if imu_initialized: # Only use IMU rotation if IMU successfully initialized
            try:
                imu_module.rotate_by_gyro(turn_angle)
                # "Rotation completed." print is inside rotate_by_gyro now
            except Exception as e:
                print(f"Error during IMU rotation: {e}")
                motor_control.send_command("0 0 0\n") # Ensure stop if rotation fails
                # Consider a simple timed turn fallback here if IMU failed mid-rotation attempt
                print("IMU rotation failed. Consider adding a timed turn fallback.")

        else:
            print("Skipping rotation due to IMU initialization failure.")
            # Add a simple timed turn fallback if IMU failed at startup
            timed_turn_duration = 1.0 # Seconds
            timed_turn_speed = 15 # Adjust speed
            print(f"Performing timed turn fallback: {timed_turn_duration}s at speed {timed_turn_speed * (1 if turn_angle > 0 else -1)}")
            motor_control.send_command(f"0 0 {timed_turn_speed * (1 if turn_angle > 0 else -1)}\n")
            time.sleep(timed_turn_duration)
            motor_control.send_command("0 0 0\n") # Stop after timed turn
            print("Timed turn fallback completed.")


    # --- Resuming movement happens after perform_avoidance returns ---
    print("Avoidance maneuver done.") # Print here after turn execution or fallback


# --- Main Execution Function ---
def main():
    """
    Main function to initialize systems and run the navigation loop.
    Uses a try...finally block for robust cleanup.
    """
    global imu_initialized, compass_initialized, lidar_initialized, ultrasound_initialized # Declare globals to modify them

    print("Starting robot systems initialization...")

    # --- Initialise all systems ---
    # Initialization calls (update global flags based on success)

    # Initialize Ultrasound sensors (using the function from your likely module)
    print("Setting up ultrasound sensors...")
    # Assuming setup_ultrasound_gpio() exists and might return a status or raise error
    try:
        # Assuming setup_ultrasound_gpio() might return True/False or just complete
        ultrasound_module.setup_ultrasound_gpio()
        ultrasound_initialized = True # Assume success if no exception
        print("Ultrasound sensors set up.")
    except Exception as e:
        print(f"Warning: Ultrasound sensors failed to set up: {e}. US obstacle detection will be disabled.")
        ultrasound_initialized = False


    # IMU Initialization and Bias Calibration
    imu_initialized = imu_module.init_imu() # init_imu prints status and returns success flag
    if imu_initialized:
        print("IMU initialized successfully.") # Print success if init_imu returns True
    else:
        print("Warning: IMU failed to initialize or calibr     E�# -*- coding: utf-8 -*-
# Filename: main.py
# Main program for robot navigation with sensors and motor control
# Orchestrates sensor inputs and motor control for obstacle avoidance.

import time
# Removed math import as it's not explicitly used in this simplified version
# Removed threading import as it's handled within modules like lidar_module
import sys # Import sys if needed elsewhere, but not strictly needed for this structure


# Import modules for robot components
import motor_control
import ultrasound_module
import imu_module
import compass_module
import lidar_module


# --- Configuration Constants ---
LIDAR_STOP_DISTANCE_CM = 27.0  # Distance Lidar should stop at (cm)
US_STOP_DISTANCE_CM = 8.0    # Distance Ultrasound should stop at (cm)
IMU_NORTH_HEADING = 0.0      # Compass heading for North (adjust if needed)
FALLBACK_TURN_ANGLE = 25.0   # Angle for fallback turns (degrees)
US_AVOID_TURN_ANGLE = 45.0   # Angle for US-triggered turns (degrees)


# --- Global State Variables ---
# Flags to track if sensors were successfully initialized
# Used to conditionally use sensors/modules
imu_initialized = False
compass_initialized = False
lidar_initialized = False
ultrasound_initialized = False # Added flag for ultrasound initialization


# State variable for alternating fallback turn direction (+1 for right, -1 for left)
last_fallback_turn_direction = 1


# --- Sensor Reading and Obstacle Detection ---
def check_for_obstacles():
    """
    Reads sensor data and determines if an obstacle is detected.
    Returns obstacle status and relevant sensor data.
    """
    global lidar_initialized, ultrasound_initialized # Need to check if sensors were initialized

    # --- Read Sensor Data ---
    # Lidar reading (default to infinity if not initialized or no valid data)
    current_lidar_distance = lidar_module.get_median_lidar_reading_cm() if lidar_initialized else float('inf')

    # Ultrasound reading (using the function from your likely ultrasound_module.py)
    # Assumes get_triggered_ultrasound_info returns a tuple: (list of triggered sensor IDs, dictionary of distances {id: distance})
    # Pass the threshold to the function
    # Handle case where ultrasound might not be initialized
    triggered_us_sensors = [] # Default empty list
    current_us_distances = {} # Default empty dictionary
    ultrasound_obstacle_status = False

    if ultrasound_initialized: # Only call US function if initialized
        try:
            # Assuming this function exists and returns (list of triggered IDs, dict of distances)
            us_info = ultrasound_module.get_triggered_ultrasound_info(US_STOP_DISTANCE_CM)
            triggered_us_sensors = us_info[0] # List of triggered sensor IDs (e.g., [0, 1])
            current_us_distances = us_info[1] # Dictionary of distances {id: distance} (e.g., {0: 5.5, 1: 7.0})
            ultrasound_obstacle_status = len(triggered_us_sensors) > 0 # Obstacle if any sensor triggered
        except Exception as e:
            print(f"Error reading ultrasound data: {e}")
            # Keep obstacle_status as False, or set to True if error should mean stop


    # --- Determine Obstacle Status ---
    # Lidar obstacle if reading is valid and below threshold (only if Lidar initialized)
    lidar_obstacle_status = (current_lidar_distance != float('inf') and current_lidar_distance < LIDAR_STOP_DISTANCE_CM) if lidar_initialized else False


    # Obstacle detected if EITHER Lidar OR Ultrasound detected one
    obstacle_detected = lidar_obstacle_status or ultrasound_obstacle_status

    # Return combined status and sensor data
    # Return US distances dict and triggered list for potential use in avoidance printing
    return obstacle_detected, lidar_obstacle_status, ultrasound_obstacle_status, current_lidar_distance, triggered_us_sensors, current_us_distances


# --- Avoidance Maneuver ---
# Updated to use the US data format from get_triggered_ultrasound_info
def perform_avoidance(lidar_triggered, us_triggered_list, us_distances_dict):
    """
    Performs an avoidance maneuver based on which sensor(s) triggered detection.
    Requires IMU to be initialized for rotation.
    us_triggered_list is a list of sensor IDs (e.g., [0, 1]), us_distances_dict is {id: distance}.
    """
    global imu_initialized, last_fallback_turn_direction, FALLBACK_TURN_ANGLE, US_AVOID_TURN_ANGLE

    print("Obstacle detected! Stopping...")
    motor_control.send_command("0 0 0\n") # Stop robot immediately

    # --- Determine Turn Angle Based on Trigger ---
    turn_angle = 0 # Default: no turn

    # Check if specific US sensors triggered individually (using IDs from the list)
    # Assuming IDs 0 and 1 are front sensors relevant for turning away
    is_us_0_triggered = 0 in us_triggered_list # Likely Physical Right Front
    is_us_1_triggered = 1 in us_triggered_list # Likely Physical Left Front
    is_back_triggered = any(us_id in us_triggered_list for us_id in [2, 3]) # Assuming IDs 2, 3 are back sensors


    # Prioritize US-specific turns if applicable and only one front US triggered
    # This logic can be refined based on robot design and desired avoidance
    if is_us_0_triggered and not is_us_1_triggered and not is_back_triggered: # Only US0 (Right Front) triggered
         print(f"US triggered (Right Front): {us_distances_dict.get(0, float('inf')):.2f}cm. Turning Left (-{US_AVOID_TURN_ANGLE:.0f}�)...")
         turn_angle = -US_AVOID_TURN_ANGLE # Turn Left
    elif is_us_1_triggered and not is_us_0_triggered and not is_back_triggered: # Only US1 (Left Front) triggered
         print(f"US triggered (Left Front): {us_distances_dict.get(1, float('inf')):.2f}cm. Turning Right (+{US_AVOID_TURN_ANGLE:.0f}�)...")
         turn_angle = US_AVOID_TURN_ANGLE # Turn Right
    # If both front US triggered OR Lidar triggered OR back US triggered -> use fallback
    elif lidar_triggered or is_back_triggered or (is_us_0_triggered and is_us_1_triggered):
        if lidar_triggered:
            print(f"LiDAR triggered. Turning...") # Lidar distance already printed in main loop
        elif len(us_triggered_list) > 0: # Any US triggered (including back or both front)
             # Print details of triggered US sensors in this case
             details = ", ".join([f"US {i}: {us_distances_dict.get(i, float('inf')):.2f}cm" for i in us_triggered_list])
             print(f"US triggered ({details}). Turning...")


        # Fallback maneuver: alternate left/right turns
        turn_angle = FALLBACK_TURN_ANGLE * last_fallback_turn_direction
        print(f"Fallback turn: {turn_angle:.0f}�")
        # Switch direction for next fallback
        last_fallback_turn_direction *= -1

    else:
        # This case should not be reached if perform_avoidance is only called when obstacle_detected is True
        print("Obstacle detected, but no specific avoidance rule matched. Stopping.")
        motor_control.send_command("0 0 0\n")
        time.sleep(1.0) # Pause
        return # Exit avoidance without turn

    # --- Execute the Turn ---
    if turn_angle != 0: # Only attempt turn if an angle was determined
        print(f"Executing rotation of {turn_angle:.0f}�...")
        if imu_initialized: # Only use IMU rotation if IMU successfully initialized
            try:
                imu_module.rotate_by_gyro(turn_angle)
                # "Rotation completed." print is inside rotate_by_gyro now
