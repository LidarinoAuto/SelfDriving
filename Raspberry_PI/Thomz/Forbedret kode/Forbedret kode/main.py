# Filename: main.py
# Main program for robot navigation with sensors and motor control

import time
import sys
from logging_utils import skriv_logg

# Import necessary modules and specific functions/classes
from Heading_Tracker import HeadingTracker
import motor_control
import imu_module
import compass_module
import ultrasound_module
import lidar_module

# Import specific functions needed by main.py and HeadingTracker
from imu_module import init_imu_system, read_imu_data, rotate_to_heading, rotate_by_gyro
from compass_module import init_compass, read_compass_data
from ultrasound_module import setup_ultrasound_gpio, get_triggered_ultrasound_info, cleanup_ultrasound_gpio
from lidar_module import start_lidar, get_median_lidar_reading_cm, stop_lidar

# --- NAVIGATION CONFIGURATION ---
FORWARD_SPEED = 100
LIDAR_STOP_DISTANCE_CM = 27.0
ULTRASOUND_STOP_DISTANCE_CM = 8.0

# Global variables
last_turn_dir = 1
heading_tracker = None
initialized_successfully = False
lidar_object = None

# --- SYSTEM INITIALIZATION ---
def initialize_systems():
    global initialized_successfully, heading_tracker, lidar_object

    skriv_logg("Starting robot systems initialization...")
    initialized_successfully = True

    # Setup Ultrasound
    try:
        skriv_logg("Setting up ultrasound sensors...")
        us_initialized = setup_ultrasound_gpio()
        if us_initialized:
            skriv_logg("Ultrasound sensors set up.")
        else:
            skriv_logg("Warning: Ultrasound sensors failed to set up.")
    except Exception as e:
        skriv_logg(f"Error setting up ultrasound sensors: {e}")

    # Initialize IMU (Gyro) and Compass system
    try:
        skriv_logg("Initializing IMU system...")
        imu_init_success = init_imu_system()
        if imu_init_success:
            skriv_logg("IMU system initialized successfully.")
        else:
            skriv_logg("IMU system initialization failed.")
            initialized_successfully = False
    except Exception as e:
        skriv_logg(f"Error during IMU system initialization: {e}")
        initialized_successfully = False

    # Initialize Compass
    try:
        skriv_logg("Initializing QMC5883L...")
        compass_init_success = init_compass()
        if compass_init_success:
            skriv_logg("QMC5883L initialization reported success.")
        else:
            skriv_logg("QMC5883L initialization reported failure.")
            initialized_successfully = False
    except Exception as e:
        skriv_logg(f"Error during QMC5883L initialization: {e}")
        initialized_successfully = False

    # Initialize Lidar
    try:
        skriv_logg("Starting Lidar...")
        lidar_object = start_lidar()
        if lidar_object:
            skriv_logg("Lidar initialized and data collection started.")
        else:
            skriv_logg("Warning: Lidar failed to initialize or connect.")
    except Exception as e:
        skriv_logg(f"Error during Lidar setup: {e}")

    # --- HEADING TRACKER SETUP ---
    try:
        if imu_module.imu_initialized and compass_module.compass_initialized:
            heading_tracker = HeadingTracker(
                imu_data_reader=read_imu_data,
                compass_data_reader=read_compass_data
            )
            skriv_logg("HeadingTracker initialized.")
        else:
            skriv_logg("Cannot create Heading Tracker: Critical sensors not initialized.")
            heading_tracker = None
            initialized_successfully = False
    except Exception as e:
        skriv_logg(f"Error creating Heading Tracker: {e}")
        heading_tracker = None
        initialized_successfully = False

    if initialized_successfully:
        skriv_logg("All critical systems initialized successfully.")
    else:
        skriv_logg("Warning: Not all systems initialized successfully. Check logs.")

# --- MAIN NAVIGATION LOOP ---
def main():
    global last_turn_dir, heading_tracker, initialized_successfully

    if not initialized_successfully or heading_tracker is None:
        skriv_logg("System not ready. Skipping main loop.")
        return

    # --- Initial Heading Alignment ---
    try:
        skriv_logg("Performing initial heading check and rotation...")
        current_heading = heading_tracker.get_heading()

        if current_heading != -1.0:
            skriv_logg(f"Current heading: {current_heading:.1f}")
            target_heading = 0.0
            error_to_north = ((target_heading - current_heading + 540.0) % 360.0) - 180.0

            initial_rotation_threshold = 5.0

            if abs(error_to_north) > initial_rotation_threshold:
                skriv_logg(f"Executing initial rotation to {target_heading:.1f} (Error: {error_to_north:.1f})...")
                imu_module.rotate_to_heading(heading_tracker, target_heading)
                skriv_logg("Initial rotation completed.")
                new_heading_after_init_rot = heading_tracker.get_heading()
                skriv_logg(f"New heading after initial rotation: {new_heading_after_init_rot:.1f}")
            else:
                skriv_logg("Initial heading is already close enough to North.")
        else:
            skriv_logg("Initial heading tracker reading failed. Skipping initial rotation check.")
    except Exception as e:
        skriv_logg(f"Error during initial heading rotation: {e}")

    # Start forward movement
    skriv_logg(f"Starting forward movement at speed {FORWARD_SPEED}...")
    motor_control.send_command(f"{FORWARD_SPEED} 0 0\n")

    try:
        while True:
            # --- READ SENSOR DATA ---
            lidar_distance = get_median_lidar_reading_cm()
            triggered_us_info = get_triggered_ultrasound_info(ULTRASOUND_STOP_DISTANCE_CM)
            triggered_us_sensors = triggered_us_info[0]
            us_distances = triggered_us_info[1]
            
            # --- UPDATE FUSED HEADING ---
            heading_tracker.update()
            current_absolute_heading = heading_tracker.get_heading()

            # --- OBSTACLE DETECTION ---
            lidar_detected = lidar_distance < LIDAR_STOP_DISTANCE_CM if lidar_distance != float('inf') else False
            us_detected = len(triggered_us_sensors) > 0

            if lidar_detected or us_detected:
                skriv_logg("--- Obstacle detected! Stopping and planning avoidance ---")
                motor_control.send_command("0 0 0\n")

                # --- OBSTACLE AVOIDANCE LOGIC ---
                turn_angle = 0
                base_turn_angle = 45
                base_fallback_angle = 25

                is_us_0 = 0 in triggered_us_sensors
                is_us_1 = 1 in triggered_us_sensors
                is_back = any(i in triggered_us_sensors for i in [2, 3])

                if is_back and not (is_us_0 or is_us_1 or lidar_detected):
                    skriv_logg("Only back sensors triggered while moving forward. Ignoring as obstacle.")
                    motor_control.send_command(f"{FORWARD_SPEED} 0 0\n")
                    time.sleep(0.05)
                    continue

                elif is_us_1 and not is_us_0:
                    turn_angle = base_turn_angle
                    skriv_logg(f"US 1 triggered: Turning Right (+{turn_angle})")

                elif is_us_0 and not is_us_1:
                    turn_angle = -base_turn_angle
                    skriv_logg(f"US 0 triggered: Turning Left ({turn_angle})")

                elif lidar_detected or (is_us_0 and is_us_1) or is_back:
                    skriv_logg("Fallback triggered (LiDAR/Both Front US/Back US). Using alternating rotation.")
                    turn_angle = base_fallback_angle * last_turn_dir
                    last_turn_dir *= -1
                    skriv_logg(f"Fallback turn: {turn_angle:+} ")

                else:
                    skriv_logg("Obstacle detected, but no specific avoidance rule matched. Remaining stopped.")
                    time.sleep(1.0)

                # Execute turn if turn_angle is set
                if turn_angle != 0:
                    try:
                        skriv_logg(f"Executing turn of {turn_angle}...")
                        imu_module.rotate_by_gyro(turn_angle)
                        skriv_logg("Avoidance maneuver completed.")
                        time.sleep(0.5)
                    except Exception as e:
                        skriv_logg(f"Error during turn execution: {e}")
                        motor_control.send_command("0 0 0\n")
                        time.sleep(1.0)

                # Resume forward movement
                skriv_logg("Resuming forward movement...")
                motor_control.send_command(f"{FORWARD_SPEED} 0 0\n")

            time.sleep(0.05)

    except KeyboardInterrupt:
        skriv_logg("Program interrupted by user (Ctrl+C).")
    except Exception as e:
        skriv_logg(f"Unexpected error occurred in main loop: {e}")
    finally:
        cleanup_systems()

# --- CLEANUP FUNCTION ---
def cleanup_systems():
    """Clean up sensor connections and stop motors."""
    skriv_logg("Cleaning up before exit...")
    motor_control.send_command("0 0 0\n")

    # Cleanup Ultrasound
    try:
        cleanup_ultrasound_gpio()
        skriv_logg("Ultrasound GPIO cleanup complete.")
    except Exception as e:
        skriv_logg(f"Error during ultrasound cleanup: {e}")

    # Cleanup Lidar
    try:
        global lidar_object
        if lidar_object is not None:
            stop_lidar()
            skriv_logg("Lidar cleanup complete.")
        else:
            skriv_logg("Lidar object was not initialized for cleanup.")
    except Exception as e:
        skriv_logg(f"Error during Lidar cleanup: {e}")

    skriv_logg("Cleanup completed. Program finished.")

# --- PROGRAM ENTRY POINT ---
if __name__ == "__main__":
    initialize_systems()

    if initialized_successfully:
        try:
            main()
        except Exception as e:
            skriv_logg(f"Program exited due to unhandled error in main loop: {e}")
        finally:
            cleanup_systems()
    else:
        skriv_logg("Critical initialization failed. Running cleanup.")
        cleanup_systems()