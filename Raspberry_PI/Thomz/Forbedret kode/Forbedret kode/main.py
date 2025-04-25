# -*- coding: utf-8 -*-

# Filename: main.py

# Main program for robot navigation with sensors and motor control
 
# --- IMPORT NECESSARY MODULES ---

import time

import sys

import threading  # Import threading if needed
 
# Import your own modules

import motor_control

import imu_module

import compass_module

import ultrasound_module

# Import the specific functions from the NEW lidar_module structure

from lidar_module import start_lidar, get_median_lidar_reading_cm, stop_lidar
 
 
# --- NAVIGATION CONFIGURATION ---

FORWARD_SPEED = 100  # Standard forward speed

# Detection thresholds (adjust as needed)

LIDAR_STOP_DISTANCE_CM = 15.0  # Stop distance for Lidar in cm (Increased slightly for safety)

ULTRASOUND_STOP_DISTANCE_CM = 8.0  # Stop distance for Ultrasound in cm (Increased slightly for safety)
 
# Global variable to keep track of the last fallback turn direction (1 for right, -1 for left)

last_turn_dir = 1
 
 
# --- SYSTEM INITIALIZATION ---

def initialize_systems():

    """Initializes all sensors and systems."""

    print("Starting robot systems initialization...")

    try:

        # Initialize Ultrasound sensors (using the updated function name)

        print("Setting up ultrasound sensors...")

        ultrasound_module.setup_ultrasound_gpio()

        print("Ultrasound sensors set up.")
 
        # Initialize IMU (assuming this version includes gyro bias calibration)

        imu_module.init_imu()
 
        # Initialize Compass (assuming this version includes upside-down compensation)

        compass_module.init_compass()
 
        # Start Lidar (using the new start function from updated lidar_module)

        lidar_initialized = start_lidar()

        if not lidar_initialized:

            print("Warning: Lidar failed to initialize or connect. Lidar data will not be available for navigation.")
 
        print("Initialization complete.")
 
    except Exception as e:

        print(f"Critical error during system initialization: {e}")

        cleanup_systems()

        sys.exit("Initialization failed.")
 
 
# --- MAIN NAVIGATION LOOP ---

def main():

    """Main navigation loop."""

    global last_turn_dir
 
    # Initial calibration or heading check (optional)

    try:

        current_heading = compass_module.read_compass()

        if current_heading != -1.0:

            print(f"Current heading: {current_heading:.1f}°")

            target_heading = 0.0

            angle_to_rotate = (target_heading - current_heading + 360) % 360

            if angle_to_rotate > 180:

                angle_to_rotate -= 360

            print(f"Rotating {angle_to_rotate:.1f}° towards North...")

            imu_module.rotate_by_gyro(angle_to_rotate)

            current_heading = compass_module.read_compass()

            print(f"New heading after rotation: {current_heading:.1f}°")

        else:

            print("Initial compass reading failed.")
 
    except Exception as e:

        print(f"Error during initial rotation or heading check: {e}")
 
    # Main navigation loop

    try:

        while True:

            # --- READ SENSOR DATA ---

            lidar_distance = get_median_lidar_reading_cm()

            triggered_ultrasound_info = ultrasound_module.get_triggered_ultrasound_info(ULTRASOUND_STOP_DISTANCE_CM)

            triggered_ultrasound_sensors = triggered_ultrasound_info[0]

            ultrasound_distances = triggered_ultrasound_info[1]
 
            # --- OBSTACLE DETECTION ---

            lidar_obstacle_detected = lidar_distance < LIDAR_STOP_DISTANCE_CM

            ultrasound_obstacle_detected = len(triggered_ultrasound_sensors) > 0
 
            if lidar_obstacle_detected or ultrasound_obstacle_detected:

                print("Obstacle detected! Stopping and planning avoidance...")

                motor_control.send_command("0 0 0\n")
 
                # --- OBSTACLE AVOIDANCE LOGIC ---

                turn_angle = 0

                base_turn_angle = 45

                base_fallback_angle = 25
 
                is_us_0_triggered = 0 in triggered_ultrasound_sensors

                is_us_1_triggered = 1 in triggered_ultrasound_sensors

                is_back_triggered = (2 in triggered_ultrasound_sensors) or (3 in triggered_ultrasound_sensors)
 
                if is_us_1_triggered and not is_us_0_triggered and not is_back_triggered:

                    turn_angle = base_turn_angle

                    print(f"US Index 1 triggered: Turning Right (+{base_turn_angle} degrees)")

                elif is_us_0_triggered and not is_us_1_triggered and not is_back_triggered:

                    turn_angle = -base_turn_angle

                    print(f"US Index 0 triggered: Turning Left (-{base_turn_angle} degrees)")

                elif lidar_obstacle_detected or is_back_triggered or (is_us_0_triggered and is_us_1_triggered):

                    print("Fallback triggered: Alternating rotation used.")

                    turn_angle = base_fallback_angle * last_turn_dir

                    last_turn_dir *= -1

                    print(f"Fallback turn by {abs(turn_angle)} degrees ({'Right' if turn_angle > 0 else 'Left'})")

                else:

                    print("Obstacle detected, but no specific avoidance rule matched. Stopping.")

                    motor_control.send_command("0 0 0\n")

                    time.sleep(1.0)
 
                if turn_angle != 0:

                    try:

                        print(f"Executing rotation of {turn_angle:.1f} degrees...")

                        imu_module.rotate_by_gyro(turn_angle)

                        print("Avoidance maneuver completed. Continuing forward.")

                    except Exception as e:

                        print(f"Error during rotation execution: {e}")

                        motor_control.send_command("0 0 0\n")

                        time.sleep(1.0)
 
            else:

                motor_control.send_command(f"{FORWARD_SPEED} 0 0\n")

                time.sleep(0.01)
 
    except KeyboardInterrupt:

        print("User interrupted the program (Ctrl+C).")

    except Exception as e:

        print(f"An unexpected error occurred in the main loop: {e}")

    finally:

        cleanup_systems()
 
 
# --- CLEANUP FUNCTION ---

def cleanup_systems():

    """Clean up sensor connections and stop motors."""

    print("Cleaning up before exit...")

    motor_control.send_command("0 0 0\n")

    ultrasound_module.cleanup_ultrasound_gpio()

    stop_lidar()

    print("Cleanup completed. Program finished.")
 
 
# --- PROGRAM ENTRY POINT ---

if __name__ == "__main__":

    initialize_systems()

    try:

        main()

    except Exception as e:

        print(f"Program terminated due to an error: {e}")

    finally:

        pass

 
