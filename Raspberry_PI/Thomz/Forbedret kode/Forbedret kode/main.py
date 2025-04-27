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
    print("Starting robot systems initialization...")

    try:
        print("Setting up ultrasound sensors...")
        # --- VIKTIG: S�rg for at setup_ultrasound_gpio i ultrasound_module stemmer med hvordan du har implementert returverdien ---
        # Hvis setup_ultrasound_gpio kaster feil ved problem (som min foresl�tte kode gjorde), fjern 'if us_initialized =' linjen og 'if us_initialized:' sjekken nedenfor.
        us_initialized = ultrasound_module.setup_ultrasound_gpio()
        if us_initialized:
            print("Ultrasound sensors set up.")
        else:
            print("Warning: Ultrasound sensors failed to set up. Navigation might be unreliable.")

        print("Initializing IMU...")
        imu_module.init_imu()
        print("IMU initialized.")

        print("Initializing QMC5883L...")
        compass_module.init_compass()
        print("QMC5883L initialized successfully.")

        print("Starting Lidar...")
        lidar_initialized = start_lidar()
        if not lidar_initialized:
            print("Warning: Lidar failed to initialize or connect.")

        print("Initialization complete.")

    except Exception as e:
        print(f"Critical error during system initialization: {e}")
        cleanup_systems()
        sys.exit(f"Initialization failed due to: {e}")

# --- MAIN NAVIGATION LOOP ---
def main():
    """Main navigation loop."""
    global last_turn_dir

    # Optional: Initial heading alignment
    try:
        print("Performing initial heading check and rotation...")
        current_heading = compass_module.read_compass()

        if current_heading != -1.0:
            print(f"Current heading: {current_heading:.1f}�")
            target_heading = 0.0
            angle_to_rotate = (target_heading - current_heading + 360) % 360
            if angle_to_rotate > 180:
                angle_to_rotate -= 360

            if abs(angle_to_rotate) > 1.0:  # Unng� rotasjon for veldig sm� vinkler
                print(f"Executing rotation of {angle_to_rotate:.1f}�...")
                imu_module.rotate_by_gyro(angle_to_rotate)
                print("Initial rotation completed.")
                current_heading = compass_module.read_compass()  # Les heading etter rotasjon
                print(f"New heading after rotation: {current_heading:.1f}�")
            else:
                print("Initial heading is already close to North.")
        else:
            print("Initial compass reading failed. Skipping initial rotation.")
    except Exception as e:
        print(f"Error during initial heading rotation: {e}")

    print("Starting continuous forward movement loop...")
    # Starter med � sende en bevegelsesskommando
    motor_control.send_command(f"{FORWARD_SPEED} 0 0\n")

    try:
        while True:
            # --- READ SENSOR DATA ---
            lidar_distance = get_median_lidar_reading_cm()
            # --- VIKTIG: get_triggered_ultrasound_info m� returnere (liste, dict) ---
            triggered_us_info = ultrasound_module.get_triggered_ultrasound_info(ULTRASOUND_STOP_DISTANCE_CM)
            triggered_us_sensors = triggered_us_info[0]  # Liste med triggede US sensor-indekser
            us_distances = triggered_us_info[1]  # Dictionary med avstander for ALLE US sensorer

            # --- DEBUG PRINTS ---
            if lidar_distance != float('inf'):
                print(f"LiDAR: {lidar_distance:.2f} cm (Threshold: {LIDAR_STOP_DISTANCE_CM:.2f})")
            elif lidar_module.lidar is not None:
                pass

            if us_distances:
                us_details = ", ".join([f"US {i}: {d:.2f}cm" for i, d in us_distances.items() if d > 0])
                print(f"US triggered: {triggered_us_sensors} | Distances: {{{us_details}}}")
            else:
                print("US triggered: [] | Distances: {}")

            # --- OBSTACLE DETECTION ---
            lidar_detected = lidar_distance < LIDAR_STOP_DISTANCE_CM if lidar_distance != float('inf') else False
            us_detected = len(triggered_us_sensors) > 0

            if lidar_detected or us_detected:
                print("--- Obstacle detected! Stopping and planning avoidance ---")
                motor_control.send_command("0 0 0\n")

                # --- OBSTACLE AVOIDANCE LOGIC ---
                turn_angle = 0
                base_turn_angle = 45
                base_fallback_angle = 25

                is_us_0 = 0 in triggered_us_sensors
                is_us_1 = 1 in triggered_us_sensors
                is_back = any(i in triggered_us_sensors for i in [2, 3])

                # --- LOGIKK FOR � SVINGE VEKK FRA HINDRING ---
                if is_us_1 and not is_us_0 and not is_back:
                    turn_angle = base_turn_angle
                    print(f"US 1 triggered: Turning Right (+{turn_angle}�)")

                elif is_us_0 and not is_us_1 and not is_back:
                    turn_angle = -base_turn_angle
                    print(f"US 0 triggered: Turning Left ({turn_angle}�)")

                # --- FALLBACK LOGIKK ---
                elif lidar_detected or is_back or (is_us_0 and is_us_1):
                    print("Fallback triggered (LiDAR/Both Front US/Back US). Using alternating rotation.")
                    turn_angle = base_fallback_angle * last_turn_dir
                    last_turn_dir *= -1
                    print(f"Fallback turn: {turn_angle:+}�")

                else:
                    print("Obstacle detected, but no specific avoidance rule matched. Remaining stopped.")
                    time.sleep(1.0)

                if turn_angle != 0:
                    try:
                        print(f"Executing turn of {turn_angle}�...")
                        imu_module.rotate_by_gyro(turn_angle)
                        print("Avoidance maneuver completed.")
                        time.sleep(0.5)
                    except Exception as e:
                        print(f"Error during turn execution: {e}")
                        motor_control.send_command("0 0 0\n")
                        time.sleep(1.0)

                print("Resuming forward movement...")
                motor_control.send_command(f"{FORWARD_SPEED} 0 0\n")

            else:
                pass

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Program interrupted by user (Ctrl+C).")
    except Exception as e:
        print(f"Unexpected error occurred in main loop: {e}")
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
    # Valgfritt: Pr�v opprydning ved start for � sikre ren tilstand (nyttig under utvikling)
    # try:
    #     cleanup_systems()
    # except Exception as e:
    #     print(f"Initial cleanup attempt failed: {e}")

    initialize_systems()

    try:
        main()
    except Exception as e:
        print(f"Program exited due to unhandled error: {e}")
    finally:
        cleanup_systems()
