# Filename: main.py
# Main program for robot navigation with sensors and motor control
from logging_utils import skriv_logg
# --- IMPORT NECESSARY MODULES ---
import time
import sys
import threading  # Import threading if needed
from Heading_Tracker import HeadingTracker
# Import your own modules
import motor_control
import imu_module
import compass_module
import ultrasound_module
import lidar_module  # Explicit import to use lidar_module.lidar

# Import the specific functions from the lidar_module
from lidar_module import start_lidar, get_median_lidar_reading_cm, stop_lidar

# --- NAVIGATION CONFIGURATION ---
FORWARD_SPEED = 100  # Standard forward speed

# Detection thresholds
LIDAR_STOP_DISTANCE_CM = 27.0  # Adjusted for robot radius (15 cm + 11.75 cm)
ULTRASOUND_STOP_DISTANCE_CM = 8.0  # Reliable stop distance for ultrasound

# Global variable to track last fallback turn direction
last_turn_dir = 1  # 1 = Right, -1 = Left

# --- SYSTEM INITIALIZATION ---
def initialize_systems():
    """Initializes all sensors and systems."""
    skriv_logg("Starting robot systems initialization...")

    try:
        skriv_logg("Setting up ultrasound sensors...")
        # --- VIKTIG: S rg for at setup_ultrasound_gpio i ultrasound_module stemmer med hvordan du har implementert returverdien ---
        # Hvis setup_ultrasound_gpio kaster feil ved problem (som min foresl tte kode gjorde), fjern 'if us_initialized =' linjen og 'if us_initialized:' sjekken nedenfor.
        us_initialized = ultrasound_module.setup_ultrasound_gpio()
        if us_initialized:
            skriv_logg("Ultrasound sensors set up.")
        else:
            skriv_logg("Warning: Ultrasound sensors failed to set up. Navigation might be unreliable.")

        skriv_logg("Initializing IMU...")
        imu_module.init_imu()
        skriv_logg("IMU initialized.")

        skriv_logg("Initializing QMC5883L...")
        compass_module.init_compass()
        skriv_logg("QMC5883L initialized successfully.")

        skriv_logg("Starting Lidar...")
        lidar_initialized = start_lidar()
        if not lidar_initialized:
            skriv_logg("Warning: Lidar failed to initialize or connect.")

        skriv_logg("Initialization complete.")
        
        # --- HEADING TRACKER SETUP --- # <--- ADD THIS BLOCK
        global heading_tracker # Declare heading tracker globally
        heading_tracker = HeadingTracker()
        heading_tracker.setup() # Initialiserer f sed heading basert p  kompass
        skriv_logg(f"Initial fused heading: {heading_tracker.get_heading():.1f} ")
        # --- END HEADING TRACKER SETUP ---

    except Exception as e:
        skriv_logg(f"Critical error during system initialization: {e}")
        cleanup_systems()
        sys.exit(f"Initialization failed due to: {e}")

# --- MAIN NAVIGATION LOOP ---
def main():
    """Main navigation loop."""
    global last_turn_dir, heading_tracker

    #Optional: Initial heading alignment
    try:
        skriv_logg("Performing initial heading check and rotation...")
        current_heading = compass_module.read_compass()

        if current_heading != -1.0:
            skriv_logg(f"Current heading: {current_heading:.1f} ")
            target_heading = 0.0
            angle_to_rotate = (target_heading - current_heading + 360) % 360
            if angle_to_rotate > 180:
                angle_to_rotate -= 360

            if abs(angle_to_rotate) > 1.0:  # Unng  rotasjon for veldig sm  vinkler
                skriv_logg(f"Executing rotation of {angle_to_rotate:.1f} ...")
                #imu_module.rotate_by_gyro(angle_to_rotate)
                imu_module.rotate_to_heading(target_heading, heading_tracker)
                skriv_logg("Initial rotation completed.")
                current_heading = compass_module.read_compass()  # Les heading etter rotasjon
                skriv_logg(f"New heading after rotation: {current_heading:.1f} ")
            else:
                skriv_logg("Initial heading is already close to North.")
        else:
            skriv_logg("Initial compass reading failed. Skipping initial rotation.")
    except Exception as e:
        skriv_logg(f"Error during initial heading rotation: {e}")

    # Sending a movement command
    motor_control.send_command(f"{FORWARD_SPEED} 0 0\n")

    try:
        while True:
            # --- READ SENSOR DATA ---
            lidar_distance = get_median_lidar_reading_cm()
            # --- VIKTIG: get_triggered_ultrasound_info m  returnere (liste, dict) ---
            triggered_us_info = ultrasound_module.get_triggered_ultrasound_info(ULTRASOUND_STOP_DISTANCE_CM)
            triggered_us_sensors = triggered_us_info[0]  # Liste med triggede US sensor-indekser
            us_distances = triggered_us_info[1]  # Dictionary med avstander for ALLE US sensorer
            
            # --- UPDATE FUSED HEADING --- #
            # N  kan du bruke fused_heading (variabelen i denne l kken) eller
            # kalle heading_tracker.get_heading() hvor som helst ellers som har tilgang til objektet.
            # Eksempel p  hvordan du ville brukt den (ikke n dvendigvis for gjeldende logikk):
            # current_absolute_heading = heading_tracker.get_heading()
            # skriv_logg(f"Current Fused Heading: {fused_heading:.2f}") # Valgfri debug-logg for f sed heading
            # --- END UPDATE FUSED HEADING ---            

            # --- DEBUG skriv_loggS ---
            #if lidar_distance != float('inf'):
             #   skriv_logg(f"LiDAR: {lidar_distance:.2f} cm (Threshold: {LIDAR_STOP_DISTANCE_CM:.2f})")
            #elif lidar_module.lidar is not None:
             #   pass

            #if us_distances:
             #   us_details = ", ".join([f"US {i}: {d:.2f}cm" for i, d in us_distances.items() if d > 0])
              #  skriv_logg(f"US triggered: {triggered_us_sensors} | Distances: {{{us_details}}}")
            #else:
             #   skriv_logg("US triggered: [] | Distances: {}")

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

                # --- LOGIKK FOR   SVINGE VEKK FRA HINDRING ---
                if is_us_1 and not is_us_0 and not is_back:
                    turn_angle = base_turn_angle
                    skriv_logg(f"US 1 triggered: Turning Right (+{turn_angle} )")

                elif is_us_0 and not is_us_1 and not is_back:
                    turn_angle = -base_turn_angle
                    skriv_logg(f"US 0 triggered: Turning Left ({turn_angle} )")

                # --- FALLBACK LOGIKK ---
                elif lidar_detected or is_back or (is_us_0 and is_us_1):
                    skriv_logg("Fallback triggered (LiDAR/Both Front US/Back US). Using alternating rotation.")
                    turn_angle = base_fallback_angle * last_turn_dir
                    last_turn_dir *= -1
                    skriv_logg(f"Fallback turn: {turn_angle:+} ")

                else:
                    skriv_logg("Obstacle detected, but no specific avoidance rule matched. Remaining stopped.")
                    time.sleep(1.0)

                if turn_angle != 0:
                    try:
                        skriv_logg(f"Executing turn of {turn_angle} ...")
                        imu_module.rotate_by_gyro(turn_angle)
                        skriv_logg("Avoidance maneuver completed.")
                        time.sleep(0.5)
                    except Exception as e:
                        skriv_logg(f"Error during turn execution: {e}")
                        motor_control.send_command("0 0 0\n")
                        time.sleep(1.0)

                skriv_logg("Resuming forward movement...")
                motor_control.send_command(f"{FORWARD_SPEED} 0 0\n")

            else:
                pass

            time.sleep(0.01)

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
    ultrasound_module.cleanup_ultrasound_gpio()
    stop_lidar()
    skriv_logg("Cleanup completed. Program finished.")

# --- PROGRAM ENTRY POINT ---
if __name__ == "__main__":
    # Valgfritt: Pr v opprydning ved start for   sikre ren tilstand (nyttig under utvikling)
    # try:
    #     cleanup_systems()
    # except Exception as e:
    #     skriv_logg(f"Initial cleanup attempt failed: {e}")

    initialize_systems()

    try:
        main()
    except Exception as e:
        skriv_logg(f"Program exited due to unhandled error: {e}")
    finally:
        cleanup_systems()
