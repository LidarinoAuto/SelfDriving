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
import imu_module # Importer hele modulen
import compass_module # Importer hele modulen
import ultrasound_module
import lidar_module

# Import specific data reading functions for passing to HeadingTracker
# Disse funksjonene M  finnes i de respektive modulene og returnere riktig data (-1.0 ved feil for kompass/heading)
from imu_module import init_imu, read_imu_data # Importer init og read fra imu
from compass_module import init_compass, read_compass_data # Importer init og read fra compass
from ultrasound_module import setup_ultrasound_gpio, get_triggered_ultrasound_info, cleanup_ultrasound_gpio # Antar disse finnes
from lidar_module import start_lidar, get_median_lidar_reading_cm, stop_lidar # Antar disse finnes

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

# --- SYSTEM INITIALIZATION ---
def initialize_systems():
    """Initializes all sensors and systems."""
    global initialized_successfully, heading_tracker, lidar_object

    skriv_logg("Starting robot systems initialization...")

    initialized_successfully = True # Start med a anta suksess

    # Setup Ultrasound
    try:
        skriv_logg("Setting up ultrasound sensors...")
        us_initialized = setup_ultrasound_gpio()
        if us_initialized:
            skriv_logg("Ultrasound sensors set up.")
        else:
            skriv_logg("Warning: Ultrasound sensors failed to set up.")
            # initialized_successfully = False # Vurder a sette til False hvis US er kritisk
    except Exception as e:
        skriv_logg(f"Error setting up ultrasound sensors: {e}")
        # initialized_successfully = False # Vurder a sette til False hvis US er kritisk


    # Initialize IMU
    try:
        skriv_logg("Initializing IMU...")
        imu_init_success = init_imu() # Kall init funksjonen importert her
        if imu_init_success:
            skriv_logg("IMU initialized.")
        else:
            skriv_logg("IMU initialization failed.")
            initialized_successfully = False # Sett flagg ved feil

    except Exception as e:
        skriv_logg(f"Error during IMU initialization: {e}")
        initialized_successfully = False


    # Initialize Compass
    try:
        skriv_logg("Initializing QMC5883L...")
        # Kall init funksjonen importert her og sjekk returverdien
        compass_init_success = init_compass() 
        if compass_init_success: # Sjekk returverdien fra init_compass
            skriv_logg("QMC5883L initialization reported success.") # Justert logg basert p forventet retur fra compass_module
        else:
            skriv_logg("QMC5883L initialization reported failure.") # Justert logg
            initialized_successfully = False # Sett flagg ved feil

    except Exception as e:
        skriv_logg(f"Error during QMC5883L initialization: {e}")
        initialized_successfully = False

    # Initialize Lidar
    try:
        skriv_logg("Starting Lidar...")
        #start_lidar() # Denne kaller din start funksjon
        # Hvis start_lidar returnerer lidar objektet, lagre det globalt
        # Din logg tyder p  at start_lidar returnerer objektet, da cleanup sender det.
        lidar_object = start_lidar()
        if lidar_object: # Sjekk om objektet ble returnert (True/False eller Objekt/None)
            skriv_logg("Lidar initialized and data collection started.") # Juster logg basert p funksjon
        else:
            skriv_logg("Warning: Lidar failed to initialize or connect.")
            # initialized_successfully = False # Vurder a sette til False hvis Lidar er kritisk
    except Exception as e:
        skriv_logg(f"Error during Lidar setup: {e}")
        # initialized_successfully = False # Vurder a sette til False hvis Lidar er kritisk
        
    # --- HEADING TRACKER SETUP ---
    # Opprett HeadingTracker objektet HER og send inn lesefunksjonene
    try:
        # Sjekk om kritisk sensor init lyktes f r vi lager trackeren
        # Sjekk de globale flaggene satt i imu_module og compass_module
        # Husk at imu_module.init_imu() og compass_module.init_compass() setter disse flaggene
        if imu_module.imu_initialized and compass_module.compass_initialized:
            # Oppretter HeadingTracker og sender inn referanser til lesefunksjonene
            heading_tracker = HeadingTracker(
                imu_data_reader=read_imu_data,  # Send funksjonen som argument
                compass_data_reader=read_compass_data # Send funksjonen som argument
            )
            # Den f rste oppdateringen/initialiseringen skjer n  i HeadingTracker.__init__
        else:
            skriv_logg("Cannot create Heading Tracker: Critical sensors not initialized.")
            heading_tracker = None # Sett til None hvis ikke opprettet
            initialized_successfully = False # Dette er kritisk, sett initialized_successfully til False
    except Exception as e:
        skriv_logg(f"Error creating Heading Tracker: {e}")
        heading_tracker = None
        initialized_successfully = False # Ogs  kritisk feil

    if initialized_successfully:
        skriv_logg("All critical systems initialized successfully.")
    else:
        skriv_logg("Warning: Not all systems initialized successfully. Check logs.")


# --- MAIN NAVIGATION LOOP ---
def main():
    """Main navigation loop."""
    global last_turn_dir, heading_tracker, initialized_successfully

    # Sjekk om systemene ble initialisert kritisk (b de flagget OG trackeren eksisterer)
    if not initialized_successfully or heading_tracker is None:
        skriv_logg("System not ready. Skipping main loop.")
        return # Avbryt main hvis ikke klar

    # --- Initial Heading Alignment ---
    try:
        skriv_logg("Performing initial heading check and rotation...")
        # Hent den nylig initialiserte fused headingen
        current_heading = heading_tracker.get_heading()


        if current_heading != -1.0: # Sjekk at vi fikk en gyldig heading ved start (Heading tracker gir aldri -1.0 hvis den ble opprettet)
            skriv_logg(f"Current heading: {current_heading:.1f} ")
            target_heading = 0.0
            # Bruk den smidige feilberegningen for a sjekke om sving trengs
            error_to_north = ((target_heading - current_heading + 540.0) % 360.0) - 180.0

            # Bruk toleransen for initial sjekk - kan v re den samme som PID bruker
            initial_rotation_threshold = 5.0 # Bruk samme verdi som heading_tolerance i rotate_to_heading

            if abs(error_to_north) > initial_rotation_threshold:
                skriv_logg(f"Executing initial rotation to {target_heading:.1f}  (Error: {error_to_north:.1f} )...")
                # Kall rotate_to_heading for a svinge til 0 grader (Nord)
                imu_module.rotate_to_heading(target_heading, heading_tracker) # Kall funksjonen, send med tracker-objektet
                skriv_logg("Initial rotation completed.")
                # Les ny heading etter initial rotasjon
                new_heading_after_init_rot = heading_tracker.get_heading()
                skriv_logg(f"New heading after initial rotation: {new_heading_after_init_rot:.1f} ")
            else:
                skriv_logg("Initial heading is already close enough to North.")
        else:
            skriv_logg("Initial heading tracker reading failed. Skipping initial rotation check.")
    except Exception as e:
        skriv_logg(f"Error during initial heading rotation: {e}")


    # Sending a movement command (Start kjoring fremover)
    skriv_logg(f"Starting forward movement at speed {FORWARD_SPEED}...")
    motor_control.send_command(f"{FORWARD_SPEED} 0 0\n")

    try:
        while True:
            # --- READ SENSOR DATA ---
            # Les Lidar og Ultrasound som for
            lidar_distance = get_median_lidar_reading_cm()
            triggered_us_info = get_triggered_ultrasound_info(ULTRASOUND_STOP_DISTANCE_CM)
            triggered_us_sensors = triggered_us_info[0]  # Liste med triggede US sensor-indekser
            us_distances = triggered_us_info[1]  # Dictionary med avstander for ALLE US sensorer

            # --- UPDATE FUSED HEADING ---
            # Viktig: Oppdater heading trackeren i hovedl kken for a holde headingen n yaktig
            heading_tracker.update()
            current_absolute_heading = heading_tracker.get_heading()
            # skriv_logg(f"Current Fused Heading: {current_absolute_heading:.2f}") # Valgfri debug-logg

            # --- DEBUG LOGS --- (Kan kommenteres ut nar alt fungerer)
            # if lidar_distance != float('inf'):
            #    skriv_logg(f"LiDAR: {lidar_distance:.2f} cm (Threshold: {LIDAR_STOP_DISTANCE_CM:.2f})")
            # # Logg US avstander hvis noen er trigget eller for spesifikke sensorer
            # if us_distances:
            #    us_details = ", ".join([f"US {i}: {d:.2f}cm" for i, d in us_distances.items()])
            #    skriv_logg(f"US triggered: {triggered_us_sensors} | Distances: {{{us_details}}}")
            
            # --- OBSTACLE DETECTION ---
            lidar_detected = lidar_distance < LIDAR_STOP_DISTANCE_CM if lidar_distance != float('inf') else False
            us_detected = len(triggered_us_sensors) > 0

            if lidar_detected or us_detected:
                skriv_logg("--- Obstacle detected! Stopping and planning avoidance ---")
                motor_control.send_command("0 0 0\n") # Stopp

                # --- OBSTACLE AVOIDANCE LOGIC --- (Bruker fortsatt rotate_by_gyro her som for)
                turn_angle = 0
                base_turn_angle = 45
                base_fallback_angle = 25

                is_us_0 = 0 in triggered_us_sensors
                is_us_1 = 1 in triggered_us_sensors
                is_back = any(i in triggered_us_sensors for i in [2, 3]) # Antar US 2 og 3 er bak

                # Unng  logikken hvis kun bakre sensorer trigger mens vi kj rer fremover
                if is_back and not (is_us_0 or is_us_1 or lidar_detected):
                    skriv_logg("Only back sensors triggered while moving forward. Ignoring as obstacle.")
                    # Kanskje rykke frem litt til for   komme fri? Eller fortsette som f r?
                    motor_control.send_command(f"{FORWARD_SPEED} 0 0\n") # Fortsett fremover
                    time.sleep(0.05) # Kort pause f r neste sjekk
                    continue # Hopp til neste loop iterasjon

                # Fortsatt i Avoidance Logic hvis ikke 'continue' ovenfor
                if is_us_1 and not is_us_0: # US 1 (typisk hgre front) trigger, US 0 (venstre front) trigger ikke
                    turn_angle = base_turn_angle # Sving hgre (positiv vinkel for rotate_by_gyro hvis h re = positiv)
                    skriv_logg(f"US 1 triggered: Turning Right (+{turn_angle} )")

                elif is_us_0 and not is_us_1: # US 0 (typisk venstre front) trigger, US 1 trigger ikke
                    turn_angle = -base_turn_angle # Sving venstre (negativ vinkel for rotate_by_gyro hvis venstre = negativ)
                    skriv_logg(f"US 0 triggered: Turning Left ({turn_angle} )")

                # --- FALLBACK LOGIKK ---
                # Lidar trigger, begge front US trigger, eller bakre US trigger (mens kj rer forover)
                elif lidar_detected or (is_us_0 and is_us_1) or is_back:
                    skriv_logg("Fallback triggered (LiDAR/Both Front US/Back US). Using alternating rotation.")
                    turn_angle = base_fallback_angle * last_turn_dir
                    last_turn_dir *= -1 # Veksle retning for neste gang
                    skriv_logg(f"Fallback turn: {turn_angle:+} ") # Logg med + for a vise retning

                else:
                    skriv_logg("Obstacle detected, but no specific avoidance rule matched. Remaining stopped.")
                    time.sleep(1.0) # Vent litt f r neste sjekk hvis ingen sving

                # Utf r svingen hvis turn_angle er satt
                if turn_angle != 0:
                    try:
                        skriv_logg(f"Executing turn of {turn_angle} ...")
                        imu_module.rotate_by_gyro(turn_angle) # Bruk rotate_by_gyro her
                        skriv_logg("Avoidance maneuver completed.")
                        time.sleep(0.5) # Liten pause etter sving
                    except Exception as e:
                        skriv_logg(f"Error during turn execution: {e}")
                        motor_control.send_command("0 0 0\n") # S rger for stopp ved feil
                        time.sleep(1.0) # Lengre pause ved feil

                # Gjenoppta kj ring fremover etter unnvikelse
                skriv_logg("Resuming forward movement...")
                motor_control.send_command(f"{FORWARD_SPEED} 0 0\n")

            # Legg til en liten pause i lkken for a unng 100% CPU-bruk
            time.sleep(0.05) # Juster denne for a balanse respons og CPU-bruk

    except KeyboardInterrupt:
        skriv_logg("Program interrupted by user (Ctrl+C).")
    except Exception as e:
        skriv_logg(f"Unexpected error occurred in main loop: {e}")
    finally:
        # --- System Cleanup ---
        cleanup_systems()
        
# --- CLEANUP FUNCTION ---
def cleanup_systems():
    """Clean up sensor connections and stop motors."""
    skriv_logg("Cleaning up before exit...")
    motor_control.send_command("0 0 0\n") # Stopp motor alltid f rst
    
    # Cleanup Ultrasound
    try:
        cleanup_ultrasound_gpio() # Kall din cleanup funksjon
        skriv_logg("Ultrasound GPIO cleanup complete.")
    except Exception as e:
        skriv_logg(f"Error during ultrasound cleanup: {e}")

    # Cleanup Lidar (Krever tilgang til lidar_object som ble opprettet i init)
    try:
        global lidar_object # Bruk det globale objektet
        # Pass p at stop_lidar kalles riktig ift. definisjonen i lidar_module.py
        # Din feilmelding tyder p  at stop_lidar() ikke tar argumenter.
        if lidar_object is not None:
            # stop_lidar(lidar_object) # Denne linjen ga feil
            stop_lidar() # <-- Kall stop_lidar UTEN argument basert p  feilmeldingen
            skriv_logg("Lidar cleanup complete.")
        else:
            skriv_logg("Lidar object was not initialized for cleanup.")
    except Exception as e:
        skriv_logg(f"Error during Lidar cleanup: {e}")

    skriv_logg("Cleanup completed. Program finished.")


# --- PROGRAM ENTRY POINT ---
if __name__ == "__main__":
    # Valgfritt: Pr v opprydning ved start for a sikre ren tilstand (nyttig under utvikling)
    # try:
    #    cleanup_systems()
    # except Exception as e:
    #    skriv_logg(f"Initial cleanup attempt failed: {e}")

    initialize_systems()

    # Kj r hovedlogikken bare hvis systemene ble initialisert ok
    if initialized_successfully:
        try:
            main()
        except Exception as e:
            skriv_logg(f"Program exited due to unhandled error in main loop: {e}")
        finally:
            cleanup_systems()
    else:
        # Hvis initialisering feilet kritisk, kj r kun opprydding
        skriv_logg("Critical initialization failed. Running cleanup.")
        cleanup_systems() # Kald opprydding uansett