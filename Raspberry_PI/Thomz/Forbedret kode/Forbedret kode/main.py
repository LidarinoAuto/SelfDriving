# Filename: main.py
# Main program for robot navigation with sensors and motor control
from logging_utils import skriv_logg
# --- IMPORT NECESSARY MODULES ---
import time
import sys
# Importer kun klassen HeadingTracker
from Heading_Tracker import HeadingTracker
# Import other modules
import motor_control
import imu_module # Importer hele modulen (inneholder init_imu_system, calibrate_gyro, read_imu_data, rotate_to_heading, rotate_by_gyro)
import compass_module # Importer hele modulen (inneholder init_compass, read_compass_data)
import ultrasound_module
import lidar_module

# Import specific functions needed by main.py
# init_imu_system is needed to initialize IMU and Gyro calibration
from imu_module import init_imu_system # <-------- BEHOLD DENNE IMPORTEN FRA IMU_MODULE

# init_compass might be called by init_imu_system, but your original code calls it separately.
# Let's keep the import and call in initialize_systems for now to match your structure.
from compass_module import init_compass # <-------- BEHOLD DENNE IMPORTEN FRA COMPASS_MODULE

# Import specific data reading functions needed by HeadingTracker (based on your Heading_Tracker.py)
# These functions must exist in your imu_module.py and compass_module.py files.
# read_imu_data should return corrected gyro Z data (e.g., degrees per second).
# read_compass_data should return heading in degrees (0-360).
from imu_module import read_imu_data # <-------- LEGG TILBAKE DENNE IMPORTEN
from compass_module import read_compass_data # <-------- LEGG TILBAKE DENNE IMPORTEN

# Import rotate_to_heading and rotate_by_gyro from imu_module
from imu_module import rotate_to_heading, rotate_by_gyro # <-------- IMPORTER DISSE FUNKSJONENE


from ultrasound_module import setup_ultrasound_gpio, get_triggered_ultrasound_info, cleanup_ultrasound_gpio # Antar disse finnes
from lidar_module import start_lidar, get_median_lidar_reading_cm, stop_lidar # Antar disse finnes

# --- CONFIGURATION CONSTANTS ---
# Define operational states
STATE_INITIALIZING = 0
STATE_NAVIGATING = 1
STATE_OBSTACLE_AVOIDANCE = 2
STATE_STOPPED = 3 # Add a stopped state

# --- NAVIGATION CONFIGURATION ---
FORWARD_SPEED = 100  # Standard forward speed

# Detection thresholds
LIDAR_STOP_DISTANCE_CM = 27.0  # Adjusted for robot radius (15 cm + 11.75 cm)
ULTRASOUND_STOP_DISTANCE_CM = 8.0  # Reliable stop distance for ultrasound

# Global variable to track last fallback turn direction
last_turn_dir = 1  # 1 = Right, -1 = Left

# Global variable for heading tracker instance
heading_tracker = None # Vil bli initialisert i initialize_systems

# Flag to indicate successful critical initialization
initialized_successfully = False

# Global variable for Lidar object (if needed directly in main, though read func is passed)
lidar_object = None # Lagre lidar objektet her hvis start_lidar returnerer det

# --- GLOBAL HEADING OFFSET ---
# Dette er den r� headingen HeadingTracker rapporterer n�r roboten peker Nord i virkeligheten.
# Funnet fra logg 17:56:23 (Initial heading 352.7 da roboten pekte Nord).
# Hvis roboten peker Nord i virkeligheten, kj�r et skript for � se
# hva heading_tracker.get_heading() rapporterer F�R offseten brukes (sjekk HeadingTracker log).
# Sett denne offseten til den rapporterte verdien n�r roboten peker Nord.
GLOBAL_HEADING_OFFSET_DEGREES = 352.7 # Start med denne verdien basert p� din siste logg


# --- SYSTEM INITIALIZATION FUNCTIONS ---
# Moved initialize_systems, obstacle_avoidance, main, cleanup_systems definitions here

def initialize_systems():
    """
    Initializes all robot systems (sensors, motors, heading tracker).
    Sets global initialized_successfully flag.
    """
    global heading_tracker, initialized_successfully, lidar_object

    skriv_logg("Starting robot systems initialization...")
    initialized_successfully = False # Assume failure until all critical systems are up

    # 1. Ultrasound Initialization (non-critical for basic navigation, but good practice)
    try:
        skriv_logg("Setting up ultrasound sensors...")
        us_initialized = setup_ultrasound_gpio()
        if us_initialized:
            skriv_logg("Ultrasound sensors set up.")
        else:
            skriv_logg("Warning: Ultrasound sensors failed to set up.")


    except Exception as e:
        skriv_logg(f"Error setting up ultrasound sensors: {e}")
        # Continue initialization, but log the error


    # 2. IMU Initialization (Critical for heading and gyro turns)
    # init_imu_system handles MPU6050 setup and gyro calibration
    # Your original code calls both separately, so we will maintain that for now.
    try:
        skriv_logg("Initializing IMU system...") # Justert logg
        # Call init_imu_system from imu_module
        imu_init_success = imu_module.init_imu_system() # <-------- BRUK DENNE FUNKSJONEN

        if imu_init_success:
            skriv_logg("IMU system initialized successfully.") # Justert logg
        else:
            skriv_logg("IMU system initialization failed.") # Justert logg
            # initialized_successfully = False # Setter flagget i feilh�ndtering under
            raise RuntimeError("IMU initialization failed.") # Kast feil for � fanges under


    except Exception as e:
        skriv_logg(f"FATAL: Error during IMU system initialization: {e}")
        # initialized_successfully = False # Setter flagget etter try/except
        cleanup_systems() # Clean up if a critical system fails early
        return # Stop initialization if IMU fails


    # 3. Compass Initialization (Critical for absolute heading)
    try:
        skriv_logg("Initializing QMC5883L...")
        # Call init_compass function imported from compass_module
        compass_init_success = compass_module.init_compass() # <-------- BRUK DENNE FUNKSJONEN

        if compass_init_success:
            skriv_logg("QMC5883L initialization reported success.")
        else:
            skriv_logg("QMC5883L initialization reported failure.")
            # initialized_successfully = False # Setter flagget i feilh�ndtering under
            raise RuntimeError("Compass initialization failed.") # Kast feil for � fanges under

    except Exception as e:
        skriv_logg(f"FATAL: Error during QMC5883L initialization: {e}")
        # initialized_successfully = False # Setter flagget etter try/except
        cleanup_systems() # Clean up if a critical system fails early
        return # Stop initialization if Compass fails

    # 4. Lidar Initialization (Critical for obstacle detection)
    # lidar_object = lidar_module.start_lidar() # start_lidar should return the lidar object if needed later
    # Assuming start_lidar handles thread creation internally and returns True/False
    lidar_initialized_ok = lidar_module.start_lidar()
    if not lidar_initialized_ok:
        skriv_logg("FATAL: Lidar failed to initialize. Cannot proceed with obstacle detection.")
        # Decide if this is FATAL or just a WARNING based on requirements
        # For now, let's make it FATAL as obstacle avoidance is key.
        cleanup_systems()
        return # Stop initialization if Lidar fails


    # 5. Heading Tracker Initialization (Critical for navigation)
    # Initialize HeadingTracker AFTER IMU and Compass are initialized
    try:
        skriv_logg("HeadingTracker: Initializing...")
        # Create HeadingTracker instance, passing sensor reading functions
        # Pass the global offset to the HeadingTracker constructor
        heading_tracker = HeadingTracker(
            imu_data_reader=imu_module.read_imu_data,
            compass_data_reader=compass_module.read_compass_data,
            global_offset_degrees=GLOBAL_HEADING_OFFSET_DEGREES # Pass offset here
        )
        # Initial heading is set inside HeadingTracker.__init__

        # Check if the HeadingTracker was able to get an initial valid heading (depends on compass_module.read_compass_data returning valid data)
        if heading_tracker is None or heading_tracker.get_heading() == -1.0:
             skriv_logg("WARNING: HeadingTracker could not get initial valid heading (likely compass issue). Proceeding without reliable heading.")
             # If reliable heading is essential, make this FATAL
             # cleanup_systems()
             # return
             # initialized_successfully = False # Consider this critical if initial heading failed
             raise RuntimeError("HeadingTracker failed to get initial heading.") # Throw error if it cannot get initial heading


    except Exception as e:
        skriv_logg(f"FATAL: Error initializing HeadingTracker: {e}")
        heading_tracker = None # Ensure it's None on error
        # initialized_successfully = False # Setting flag after try/except
        cleanup_systems() # Clean up if a critical system fails early
        return # Stop initialization if HeadingTracker fails

    # If we reached here, all critical systems (IMU, Lidar, HeadingTracker) initialized
    # Set initialized_successfully to True ONLY if all steps above completed without returning
    initialized_successfully = True # <-------- SETT FLAGGET HER HVIS ALT LYKKES
    skriv_logg("All critical systems initialized successfully.")

    # Optional: Perform initial check and rotation to North (0 degrees)
    # Only do this if HeadingTracker was successfully initialized with a valid heading
    if initialized_successfully and heading_tracker is not None and heading_tracker.get_heading() != -1.0:
        skriv_logg("Performing initial heading check and rotation...")
        current_heading = heading_tracker.get_heading() # This is the adjusted heading
        skriv_logg(f"Current heading: {current_heading:.1f}\t")

        # Define target heading (North)
        target_heading = 0.0 # North

        # Check if already close enough to North (within heading_tolerance)
        # Need to handle wrap-around when calculating difference for the check
        error_at_start = target_heading - current_heading
        if error_at_start > 180:
             error_at_start -= 360
        elif error_at_start < -180:
             error_at_start += 360

        # Use imu_module.heading_tolerance for the check
        if abs(error_at_start) > imu_module.heading_tolerance:
            skriv_logg(f"Initial heading is {current_heading:.1f}, outside tolerance {imu_module.heading_tolerance:.1f} from {target_heading:.1f}. Executing initial rotation to {target_heading:.1f} (Error: {error_at_start:.1f} )...")
            # Add debug log before calling rotate_to_heading
            skriv_logg(f"Calling rotate_to_heading with heading_tracker, target_heading={target_heading}")
            # Kall rotate_to_heading for a svinge til 0 grader (Nord)
            # Pass HeadingTracker instansen til rotate_to_heading
            rotation_successful = rotate_to_heading(heading_tracker, target_heading) # <-------- KALL rotate_to_heading FUNKSJONEN IMPORTERT FRA imu_module
            # Add debug log after calling rotate_to_heading
            skriv_logg("Returned from rotate_to_heading.")

            if rotation_successful:
                skriv_logg("Initial rotation completed.")
                # Get new heading after rotation (this is the adjusted heading)
                new_heading_after_init_rot = heading_tracker.get_heading()
                skriv_logg(f"New heading after initial rotation: {new_heading_after_init_rot:.1f} ")
            else:
                skriv_logg("WARNING: Initial rotation failed or timed out.")
                # Decide how to handle rotation failure (stop, try again, proceed anyway)
                # For now, we proceed, but robot might not be facing North.
        else:
            skriv_logg("Initial heading is already close enough to North.")
            skriv_logg(f"Current heading: {current_heading:.1f}\t")
    elif not initialized_successfully:
         skriv_logg("Skipping initial heading check and rotation due to initialization failures.")
    else: # heading_tracker is None or heading_tracker.get_heading() is -1.0
         skriv_logg("Skipping initial heading check and rotation as HeadingTracker is not providing a valid heading.")

# --- OBSTACLE AVOIDANCE LOGIC ---
def obstacle_avoidance():
    """
    Checks for obstacles using Lidar and Ultrasound sensors and performs avoidance maneuvers.
    Returns True if an obstacle was detected and avoided, False otherwise.
    """
    global last_turn_dir, heading_tracker # Use global variables

    # Ensure heading_tracker is available before reading heading
    if heading_tracker is None or heading_tracker.get_heading() == -1.0:
        skriv_logg("Warning: HeadingTracker not available or invalid heading in obstacle_avoidance. Cannot perform heading-aware avoidance.")
        # Decide how to handle this: skip avoidance, use only distance sensors, or stop.
        # For now, we'll try using only distance sensors and fallback turns.

    # 1. Check Lidar (primary long-range sensor)
    lidar_distance = get_median_lidar_reading_cm()
    lidar_detected = lidar_distance != float('inf') and lidar_distance < LIDAR_STOP_DISTANCE_CM

    # 2. Check Ultrasound (secondary short-range sensors)
    try:
        # get_triggered_ultrasound_info returns (triggered_sensor_list, distance_dict)
        # The error was that main.py was calling check_all_ultrasound_sensors,
        # but the function name in ultrasound_module.py is get_triggered_ultrasound_info.
        # We change the call in main.py to match the actual function name.
        triggered_us_info = ultrasound_module.get_triggered_ultrasound_info(ULTRASOUND_STOP_DISTANCE_CM) # <-------- CHANGED FUNCTION CALL HERE
        triggered_us_sensors = triggered_us_info[0]  # List of triggered US sensor IDs
        us_distances = triggered_us_info[1]  # Dictionary of distances for ALL US sensors
        us_detected = len(triggered_us_sensors) > 0
    except AttributeError:
         # This should not happen if the function name is correct now
         skriv_logg("Error: ultrasound_module has no attribute 'get_triggered_ultrasound_info'. Check function name.")
         triggered_us_sensors = []
         us_distances = {}
         us_detected = False
    except Exception as e:
        skriv_logg(f"Error reading ultrasound sensors: {e}")
        triggered_us_sensors = []
        us_distances = {}
        us_detected = False # Assume no detection on error

    if lidar_detected or us_detected:
        skriv_logg("--- Obstacle detected! Stopping and planning avoidance ---")
        motor_control.send_command("0 0 0\n") # Stop immediately

        # Analyze which sensors triggered to decide avoidance strategy
        # US sensor IDs: 0 (Front Left), 1 (Front Right), 2 (Back Left), 3 (Back Right)

        turn_angle = 0 # Initialize turn angle
        base_turn_angle = 45 # Default angle for US-based turn
        base_fallback_angle = 25 # Default angle for fallback turn

        is_us_0 = 0 in triggered_us_sensors
        is_us_1 = 1 in triggered_us_sensors
        is_back = any(i in triggered_us_sensors for i in [2, 3]) # Assume US 2 and 3 are back

        # Avoid the logic if only back sensors trigger while we are trying to move forward
        # This prevents stopping if we just backed into something.
        # Need to know the robot's current movement state to do this properly.
        # For now, let's keep the simple check based on triggered sensors.
        # if is_back and not (is_us_0 or is_us_1 or lidar_detected):
        #     # Assuming this check is within the main loop where we know we are moving forward
        #     # If this function is called from obstacle_avoidance state,
        #     # we need to re-evaluate the condition.
        #     # For now, let's assume the robot is moving forward when this is called from the main loop.
        #     skriv_logg("Only back sensors triggered while moving forward. Ignoring as obstacle.")
        #     # Resume forward movement and hope the obstacle is transient or behind us now.
        #     # motor_control.send_command(f"{FORWARD_SPEED} 0 0\n")
        #     # time.sleep(0.05) # Short pause
        #     return False # Not considered a detected obstacle that requires avoidance

        # If not just back sensors triggered, proceed with avoidance logic
        if is_us_1 and not is_us_0: # US 1 (typically right front) triggered, US 0 (left front) did not
            turn_angle = base_turn_angle # Turn Right (positive angle for rotate_by_gyro)
            skriv_logg(f"US 1 triggered: Turning Right (+{turn_angle} )")

        elif is_us_0 and not is_us_1: # US 0 (typically left front) triggered, US 1 did not
            turn_angle = -base_turn_angle # Turn Left (negative angle for rotate_by_gyro)
            skriv_logg(f"US 0 triggered: Turning Left ({turn_angle} )")

        # --- FALLBACK LOGIC ---
        # Lidar triggered, both front US triggered, or back US triggered (while moving forward)
        elif lidar_detected or (is_us_0 and is_us_1) or is_back:
            skriv_logg("Fallback triggered (LiDAR/Both Front US/Back US). Using alternating rotation.")
            turn_angle = base_fallback_angle * last_turn_dir # Determine turn angle and direction
            last_turn_dir *= -1 # Switch direction for the next fallback turn
            skriv_logg(f"Fallback turn: {turn_angle:+} ") # Log with + for clarity


        else:
            skriv_logg("Obstacle detected, but no specific avoidance rule matched. Remaining stopped.")
            # Maybe add a small timeout here before returning False
            time.sleep(1.0) # Wait a bit before the next check
            return True # Still considered an obstacle event

        # Execute the turn if a turn angle was determined
        if turn_angle != 0:
            try:
                skriv_logg(f"Executing turn of {turn_angle} ...")
                # Call rotate_by_gyro for relative turn using gyro
                # Make sure rotate_by_gyro is imported from imu_module
                rotate_by_gyro(turn_angle) # <-------- KALL rotate_by_gyro FUNKSJONEN IMPORTERT FRA imu_module
                skriv_logg("Avoidance maneuver completed.")
                time.sleep(0.5) # Small pause after turning
            except Exception as e:
                skriv_logg(f"Error during turn execution: {e}")
                motor_control.send_command("0 0 0\n") # Ensure stop on error
                time.sleep(1.0) # Longer pause on error

        # Resuming forward movement happens in the main loop after this function returns True
        skriv_logg("Resuming forward movement...")

        return True # Indicate that an obstacle was detected and avoidance was attempted

    # No obstacle detected by Lidar or Ultrasound
    return False

# --- MAIN PROGRAM LOOP ---
def main():
    """
    Main loop for robot navigation.
    """
    skriv_logg("Starting main navigation loop.")
    current_state = STATE_NAVIGATING # Start in navigating state

    try:
        while True:
            # State Machine logic
            if current_state == STATE_NAVIGATING:
                # Check for obstacles
                if obstacle_avoidance():
                    # If obstacle avoidance happens, we remain in NAVIGATING state
                    # and simply resume forward movement after avoidance.
                    # Or you could transition to STATE_OBSTACLE_AVOIDANCE if avoidance
                    # is a more complex, multi-step process.
                    pass # Stay in navigating state and resume below

                # If no obstacle was detected, continue moving forward
                # Ensure we are moving forward if not in avoidance
                # motor_control.move_forward() # This sends command repeatedly
                # Better to send forward command once and let state manage
                # Only move forward if system initialized successfully and HeadingTracker is working
                if initialized_successfully and heading_tracker is not None and heading_tracker.get_heading() != -1.0:
                    # Optional: Add navigation logic here (e.g., check heading, adjust course)
                    # current_absolute_heading = heading_tracker.get_heading() # <-------- GET ADJUSTED HEADING HERE
                    # if current_absolute_heading != -1.0:
                    #     skriv_logg(f"Current adjusted heading: {current_absolute_heading:.1f}")
                        # Add logic to correct course if drifting from target heading (e.g., North)
                        # This would be a second, slower PID loop for course correction

                    skriv_logg(f"Moving forward at speed {FORWARD_SPEED}...")
                    motor_control.send_command(f"{FORWARD_SPEED} {FORWARD_SPEED} {0.0:.2f}\n")
                else:
                     # If not initialized or heading is invalid, stop and log
                     skriv_logg("Not initialized successfully or heading invalid. Stopping.")
                     motor_control.stop_robot()
                     time.sleep(1.0) # Pause before checking again
                     # Maybe transition to a stopped state or attempt re-initialization


                pass # Currently just moves forward until obstacle


            elif current_state == STATE_OBSTACLE_AVOIDANCE:
                # Logic for complex obstacle avoidance if needed
                # For now, avoidance happens within obstacle_avoidance() and returns to NAVIGATING state implicitly.
                pass

            elif current_state == STATE_STOPPED:
                 # Robot is stopped, waiting for external command or condition
                 motor_control.stop_robot()
                 pass # Stay in stopped state until state is changed


            # Add a small delay in the main loop to avoid high CPU usage
            time.sleep(0.05) # Adjust as needed - reduced from 0.1 for potentially faster loop updates


    except KeyboardInterrupt:
        skriv_logg("Program interrupted by user (Ctrl+C).")
        current_state = STATE_STOPPED # Transition to stopped state

    except Exception as e:
        skriv_logg(f"An unexpected error occurred in the main loop: {e}")
        current_state = STATE_STOPPED # Transition to stopped state

    finally:
        # Ensure cleanup is called on exit
        cleanup_systems()


# --- SYSTEM CLEANUP ---
def cleanup_systems():
    """Clean up sensor connections, stop motors, etc."""
    skriv_logg("Cleaning up before exit...")

    # 1. Stop Motors
    skriv_logg("Stopping motors...")
    motor_control.send_command("0 0 0\n") # Send stop command to ESP32

    # 2. Clean up Ultrasound GPIO
    try:
        # Assuming ultrasound_module has a cleanup_ultrasound_gpio() function
        ultrasound_module.cleanup_ultrasound_gpio() # Call your cleanup function
        skriv_logg("Ultrasound GPIO cleanup complete.")
    except Exception as e:
        skriv_logg(f"Error during ultrasound cleanup: {e}")

    # 3. Stop Lidar thread and cleanup (assuming lidar_module has a stop_lidar function)
    try:
        global lidar_object # Use the global lidar object if stop_lidar needs it
        # Ensure stop_lidar is called correctly based on its definition in lidar_module.py
        # Your previous error message suggests stop_lidar() doesn't take arguments.
        # If lidar_object is checked inside stop_lidar, just call it.
        # If not, check here:
        if lidar_object is not None:
            # Based on previous error message: stop_lidar(lidar_object) was wrong.
            # Call stop_lidar() without arguments if that's how it's defined.
            lidar_module.stop_lidar() # <-- Call stop_lidar WITHOUT argument
            # You might need a small delay for the lidar thread to finish
            if lidar_module.lidar_thread_instance and lidar_module.lidar_thread_instance.is_alive():
                 skriv_logg("Waiting for Lidar thread to join...")
                 # Give the thread a short time to finish
                 lidar_module.lidar_thread_instance.join(timeout=2.0)
                 if lidar_module.lidar_thread_instance.is_alive():
                      skriv_logg("Warning: Lidar thread did not finish gracefully within timeout.")
                 else:
                      skriv_logg("Lidar thread joined successfully.")

            skriv_logg("Lidar cleanup complete.")
        else:
            # If lidar_object was not initialized, still try to call stop_lidar
            # in case it handles some global cleanup.
            skriv_logg("Lidar object was not initialized for cleanup.")
            try:
                 lidar_module.stop_lidar()
            except Exception as e:
                 skriv_logg(f"Error during Lidar stop attempt when object was None: {e}")


    except Exception as e:
        skriv_logg(f"Error during Lidar cleanup: {e}")

    # 4. Add cleanup for IMU/Compass if they have specific cleanup methods (unlikely for these sensors)
    # For MPU6050 and QMC5883L, usually no specific cleanup is needed beyond stopping reads.

    # 5. Add cleanup for the serial connection to ESP32 if it's managed globally in motor_control.py
    # Assuming motor_control.py manages the serial connection internally and it's closed on script exit.
    # If you need explicit close: motor_control.close_serial() # (If you add this function)


    skriv_logg("Cleanup completed. Program finished.")


# --- PROGRAM ENTRY POINT ---
if __name__ == "__main__":
    # Optional: Try cleanup at start for a clean state (useful during development)
    # try:
    #    cleanup_systems()
    # except Exception as e:
    #    # FIX: Removed unnecessary backslashes causing SyntaxError
    #    skriv_logg(f"Initial cleanup attempt failed: {e}")
    # except Exception as e: # Removed duplicate except block causing error
    #     pass # This except block was empty and redundant


    initialize_systems()

    # Run the main logic only if critical systems were initialized successfully
    if initialized_successfully:
        try:
            main() # <--- Now main() is defined before this block
        except Exception as e:
            # FIX: Removed unnecessary backslashes causing SyntaxError
            skriv_logg(f"Program exited due to unhandled error in main loop: {e}")
    else:
        skriv_logg("System initialization failed. Main loop will not run.")
        # Ensure cleanup is called if initialization failed. It's called at the end of initialize_systems now.

    # Ensure cleanup is called if main loop finished without exception (e.g., state change to stopped and break)
    # Or if initialization failed and we didn't exit explicitly
    # The finally block in main() calls cleanup_systems().
    # If initialize_systems() fails before main is called, cleanup_systems() is called at the end of initialize_systems.
    pass # Cleanup is handled by the calls within initialize_systems and main's finally block
