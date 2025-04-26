# Filename: main.py
# Main program for robot navigation with sensors and motor control

# --- IMPORT NECESSARY MODULES ---
import time
import sys
import threading # Import threading if needed

# Import your own modules
import motor_control
import imu_module
import compass_module
import ultrasound_module
# Import the specific functions from the NEW lidar_module structure
from lidar_module import start_lidar, get_median_lidar_reading_cm, stop_lidar


# --- NAVIGATION CONFIGURATION ---
FORWARD_SPEED = 100 # Standard forward speed
# Detection thresholds (adjust as needed)
LIDAR_STOP_DISTANCE_CM = 15.0 # Stop distance for Lidar in cm (Increased slightly for safety)
ULTRASOUND_STOP_DISTANCE_CM = 8.0 # Stop distance for Ultrasound in cm (Increased slightly for safety)
# Adjusted these thresholds slightly based on common sensor performance and to avoid triggering on very small variations

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
        # start_lidar handles connection and thread start, returns True if successful
        lidar_initialized = lidar_module.start_lidar()
        if not lidar_initialized:
            print("Warning: Lidar failed to initialize or connect. Lidar data will not be available for navigation.")
            # Lidar functions in lidar_module will return float('inf') if not initialized/running

        print("Initialization complete.")

    except Exception as e:
        print(f"Critical error during system initialization: {e}")
        cleanup_systems() # Attempt to clean up on critical init error
        sys.exit("Initialization failed.") # Exit the program


# --- MAIN NAVIGATION LOOP ---
def main():
    """Main navigation loop."""
    global last_turn_dir

    # Initial calibration or heading check (optional)
    try:
        current_heading = compass_module.read_compass()
        if current_heading != -1.0: # Check if compass reading was successful
            print(f"Current heading: {current_heading:.1f}�")
            # Example: Turn towards North (0 degrees) - adjust target as needed
            target_heading = 0.0
            # Calculate shortest angle to turn towards target heading
            angle_to_rotate = (target_heading - current_heading + 360) % 360
            if angle_to_rotate > 180:
                angle_to_rotate -= 360
            print(f"Rotating {angle_to_rotate:.1f}� towards North...")
            # rotate_by_gyro handles the rotation using IMU
            imu_module.rotate_by_gyro(angle_to_rotate)
            # Re-read heading after rotation for verification
            current_heading = compass_module.read_compass()
            print(f"New heading after rotation: {current_heading:.1f}�")
        else:
            print("Initial compass reading failed.")

    except Exception as e:
        print(f"Error during initial rotation or heading check: {e}")
        # Decide how to handle this failure (e.g., proceed without precise heading)


    # Main navigation loop
    try:
        while True:
            # --- READ SENSOR DATA ---
            # Read Lidar data (using the new function, returns float('inf') if Lidar is not working)
            lidar_distance = lidar_module.get_median_lidar_reading_cm()

            # Read Ultrasound data (using the updated function)
            # Returns a tuple: (list of triggered sensor IDs, dictionary of distances)
            triggered_ultrasound_info = ultrasound_module.get_triggered_ultrasound_info(ULTRASOUND_STOP_DISTANCE_CM)
            triggered_ultrasound_sensors = triggered_ultrasound_info[0] # List of triggered sensor indices
            ultrasound_distances = triggered_ultrasound_info[1] # Dictionary of distances

            # Read compass heading (optional in main loop, for logging or complex navigation)
            # current_heading = compass_module.read_compass()
            # if current_heading != -1.0:
            #     print(f"Compass: {current_heading:.1f}� ({compass_module.get_compass_direction(current_heading)})")


            # --- OBSTACLE DETECTION ---
            # Check if Lidar OR any Ultrasound sensor detected an obstacle below threshold
            lidar_obstacle_detected = lidar_distance < LIDAR_STOP_DISTANCE_CM
            ultrasound_obstacle_detected = len(triggered_ultrasound_sensors) > 0

            # If any obstacle is detected by Lidar or Ultrasound
            if lidar_obstacle_detected or ultrasound_obstacle_detected:
                print("Obstacle detected! Stopping and planning avoidance...")
                motor_control.send_command("0 0 0\n") # Stop motors immediately

                # --- OBSTACLE AVOIDANCE LOGIC ---
                turn_angle = 0 # Default: no turn
                base_turn_angle = 45 # Angle for specific US triggers (adjust as needed, e.g., 25, 30)
                base_fallback_angle = 25 # Angle for fallback triggers (adjust as needed)

                # Check specific Ultrasound triggers first
                # Assuming Index 0 = Front Left (FV), Index 1 = Front Right (FH), Index 2 = Back Left (BV), Index 3 = Back Right (BH)
                # based on how they triggered in previous logs. Adjust indices if your physical wiring is different.
                is_us_0_triggered = 0 in triggered_ultrasound_sensors # Likely Physical Right Front
                is_us_1_triggered = 1 in triggered_ultrasound_sensors # Likely Physical Left Front
                is_back_triggered = (2 in triggered_ultrasound_sensors) or (3 in triggered_ultrasound_sensors)


                # --- LOGIC TO TURN AWAY FROM OBSTACLE ---
                # If only Sensor 1 (Physical Left Front?) triggered -> Turn Right
                if is_us_1_triggered and not is_us_0_triggered and not is_back_triggered: # Checks index 1 only
                    turn_angle = base_turn_angle # Positive angle results in Right turn
                    print(f"US Index 1 (Likely Physical Left Front) triggered: Turning Right (+{base_turn_angle} degrees)")

                # If only Sensor 0 (Physical Right Front?) triggered -> Turn Left
                elif is_us_0_triggered and not is_us_1_triggered and not is_back_triggered: # Checks index 0 only
                    turn_angle = -base_turn_angle # NEGATIVE angle results in Left turn
                    print(f"US Index 0 (Likely Physical Right Front) triggered: Turning Left (-{base_turn_angle} degrees)")

                # --- FALLBACK LOGIC ---
                # If no specific US rule matched, or if Lidar/Back US/Both Front US triggered, use fallback
                # This covers: Lidar detection, Back US detection, Both Front US detection
                elif lidar_obstacle_detected or is_back_triggered or (is_us_0_triggered and is_us_1_triggered):
                     print("Fallback triggered (LiDAR/Both Front US/Back US): Alternating rotation used.")
                     # Determine fallback angle and direction (alternating)
                     turn_angle = base_fallback_angle * last_turn_dir
                     # Reverse direction for next fallback turn
                     last_turn_dir *= -1
                     print(f"Fallback turn by {abs(turn_angle)} degrees ({'Right' if turn_angle > 0 else 'Left'})")

                else:
                     # This case should not be reached if any obstacle was detected,
                     # but included as a safety net.
                     print("Obstacle detected, but no specific avoidance rule matched. Stopping.")
                     motor_control.send_command("0 0 0\n")
                     time.sleep(1.0) # Pause


                # If a turn angle was determined (either specific US or fallback)
                if turn_angle != 0:
                    try:
                        # Execute the rotation using IMU
                        print(f"Executing rotation of {turn_angle:.1f} degrees...")
                        imu_module.rotate_by_gyro(turn_angle)
                        print("Avoidance maneuver completed. Continuing forward.")
                    except Exception as e:
                         print(f"Error during rotation execution: {e}")
                         # Decide how to handle rotation failure - stop, retry, etc.
                         motor_control.send_command("0 0 0\n") # Ensure stop if rotation fails
                         time.sleep(1.0) # Pause after error

            # --- CONTINUE FORWARD MOVEMENT ---
            else:
                # No obstacle detected, continue moving forward
                # print(f"Path clear. Moving forward ({FORWARD_SPEED}). Lidar: {lidar_distance:.2f}cm, US triggered: {triggered_ultrasound_sensors}") # Optional debug print
                motor_control.send_command(f"{FORWARD_SPEED} 0 0\n")
                time.sleep(0.01) # Small delay in main loop


    except KeyboardInterrupt:
        print("User interrupted the program (Ctrl+C).")
    except Exception as e:
        print(f"An unexpected error occurred in the main loop: {e}")
    finally:
        # --- CLEAN UP SYSTEMS ---
        # Ensure all systems are stopped cleanly on exit
        cleanup_systems()


# --- CLEANUP FUNCTION ---
def cleanup_systems():
    """Clean up sensor connections and stop motors."""
    print("Cleaning up before exit...")
  ,q# -*- coding: utf-8 -*-
# Filename: main.py
# Main program for robot navigation with sensors and motor control

# --- IMPORT NECESSARY MODULES ---
import time
import sys
import threading # Import threading if needed

# Import your own modules
import motor_control
import imu_module
import compass_module
import ultrasound_module
# Import the specific functions from the NEW lidar_module structure
from lidar_module import start_lidar, get_median_lidar_reading_cm, stop_lidar


# --- NAVIGATION CONFIGURATION ---
FORWARD_SPEED = 100 # Standard forward speed
# Detection thresholds (adjust as needed)
LIDAR_STOP_DISTANCE_CM = 15.0 # Stop distance for Lidar in cm (Increased slightly for safety)
ULTRASOUND_STOP_DISTANCE_CM = 8.0 # Stop distance for Ultrasound in cm (Increased slightly for safety)
# Adjusted these thresholds slightly based on common sensor performance and to avoid triggering on very small variations

# Global variable to keep track of the last fallback turn direction (1 for right, -1 for left)
last_turn_dir = 1

# --- SYSTEM INITIALIZATION ---
def initialize_systems():
    """Initialize