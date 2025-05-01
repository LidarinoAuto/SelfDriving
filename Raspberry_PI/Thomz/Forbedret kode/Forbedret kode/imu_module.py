# Filename: imu_module.py - KORRIGERT VERSJON
import smbus    # For I2C communication
import time
import math
from logging_utils import skriv_logg
import motor_control # Import motor_control module (antar denne inneholder send_command)
import compass_module # Import compass_module (antar denne inneholder compass_initialized og read_compass_data)
# --- SJEKK/TILPASS DETTE ---
from Heading_Tracker import HeadingTracker 


# --- Configuration ---
IMU_ADDRESS = 0x68      # I2C address (0x68 or 0x69) - Check with i2cdetect -y 1
PWR_MGMT_1 = 0x6B       # Power Management Register 1
GYRO_ZOUT_H = 0x47      # High byte of Z-axis gyro data
GYRO_SCALE_FACTOR = 1.0 / 131.0 # Or adjust based on your GYRO_CONFIG

# --- Global Variables ---
bus = None
integrated_angle = 0.0    # Integrated angle for current rotation (used by rotate_by_gyro)
last_time_gyro = 0.0         # Time of the last IMU reading for integration (for rotate_by_gyro)
gyro_bias_z = 0.0       # Calculated Z-axis gyro bias
bias_calibration_done = False # Flag
imu_initialized = False # Added initialization flag

# --- PID & Rotasjon Kontroll Konfigurasjon ---
Kp_heading = 0.08   # Proportional gain for heading control (Adjust this!)
Kd_heading = 1.0   # Derivative gain for damping oscillations (Adjust this!)
# Ki_heading = 0.0 # Integral gain (ikke i bruk pga oscillasjonsproblemer)
heading_tolerance = 5.0 # Degrees tolerance to consider target reached (Adjust!)
# Interne PID skaleringskonstanter (Arbitr r skala 0-50/8-50)
base_speed_heading = 50 
min_rotate_speed_heading = 8 
# Kalibrert Verdi: Maksimal rotasjonshastighet i deg/s ved base_speed_heading kommando (50.0)
MAX_ROBOT_ROTATION_DEGPS_AT_BASE_SPEED = 37.80 # <--- SETT INN DIN FAKTISKE M LTE VERDI (POSITIV)!
# Minimum rotasjonshastighet i deg/s som trengs for at roboten skal bevege seg p litelig
MIN_ROBOT_ROTATION_DEGPS = (min_rotate_speed_heading / base_speed_heading) * MAX_ROBOT_ROTATION_DEGPS_AT_BASE_SPEED

# --- Initialisering av IMU ---
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

        # Set the initial time after calibration for integration (for rotate_by_gyro)
        last_time_gyro = time.time()
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
def read_raw_gyro_z():
    """Reads raw Z-axis gyroscope data from the IMU."""
    global bus
    if bus is None:
        # skriv_logg("Warning: Attempted to read raw gyro before bus initialized.") # Optional debug log
        return 0 # Return 0 if bus is not initialized or error occurred earlier

    try:
        # Read high and low byte for Z-axis gyro
        gyro_z_h = bus.read_byte_data(IMU_ADDRESS, GYRO_ZOUT_H)
        gyro_z_l = bus.read_byte_data(IMU_ADDRESS, GYRO_ZOUT_H + 1)
        # Combine bytes to a signed 16-bit integer
        raw_value = (gyro_z_h << 8) | gyro_z_l
        if raw_value > 32767: # Handle negative numbers (2's complement)
            raw_value -= 65536

        return float(raw_value)
    
    except Exception:
        return 0.0
    
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
    global integrated_angle, last_time_gyro, imu_initialized, bias_calibration_done

    # Ensure IMU is initialized and calibrated before attempting rotation
    if not imu_initialized or not bias_calibration_done:
        skriv_logg("Cannot execute rotate_by_gyro: IMU not initialized or calibrated.")
        # Assuming motor_control is the imported module
        motor_control.send_command(f"0.0 0.0 {0.0:.2f}\n") # Ensure motors are stopped
        return # Exit function if IMU is not ready
    
    # Reset integrated angle BEFORE starting THIS rotation
    integrated_angle = 0.0
    last_time_gyro = time.time() # Get start time for this rotation

    skriv_logg(f"Starting rotation (gyro): {target_angle_degrees:.2f} ") # Added skriv_logg with degree symbol

    # --- Rotation Control Parameters (Adjust these for tuning) ---
    Kp_speed = 0.4 # Proportional gain for speed control (Lower = slower, Higher = faster turn)
    angle_tolerance = 1.0 # Stop when within +/- 1.0 degrees of target (increased slightly for stability)
    base_speed = 18 # Max/Base motor speed for rotation (kept at 18) - THIS IS FOR rotate_by_gyro ONLY
    min_rotate_speed = 5 # Minimum motor speed to prevent stalling near target (kept at 5) - THIS IS FOR rotate_by_gyro ONLY
    # Set a timeout for rotation to prevent infinite loops if IMU fails mid-turn
    rotation_timeout = 10.0 # seconds (Adjust as needed)
    start_rotation_time = time.time()
    
    # Rotation control loop based on integrated angle
    while abs(target_angle_degrees - integrated_angle) > angle_tolerance and (time.time() - start_rotation_time) < rotation_timeout:
        current_time = time.time()
        # Calculate time difference since last reading
        dt = max(current_time - last_time_gyro, 0.0001)
        last_time_gyro = current_time # Update last time for the next iteration

        # Read compensated angular velocity (degrees/sec)
        current_velocity = read_imu_data()
        # Integrate velocity over time to get current angle
        integrated_angle += current_velocity * dt
        # Calculate error (remaining angle needed)
        error = target_angle_degrees - integrated_angle

        # Calculate motor speed command based on proportional control
        rotate_speed_command = -Kp_speed * error # <--- BEHOLD MINUS HER for rotate_by_gyro
        # Apply speed limits and minimum speed based on the calculated command's magnitude.
        abs_speed_command = abs(rotate_speed_command)
        
        # Clamp between min_rotate_speed and base_speed
        if abs_speed_command < min_rotate_speed:
            # If below min, use min speed magnitude
            clamped_abs_speed = min_rotate_speed
        elif abs_speed_command > base_speed:
            # If above max, use base speed magnitude
            clamped_abs_speed = base_speed
        else:
            # If within range, use calculated speed magnitude
            clamped_abs_speed = abs_speed_command
            
        # Reapply the original sign and prepare command for ESP32
        rotate_command_for_esp = math.copysign(clamped_abs_speed, rotate_speed_command)
        # --- SEND KOMMANDO TIL ESP32 ---
        motor_control.send_command(f"0.0 {0.0:.2f} {rotate_command_for_esp:.2f}\n")


        time.sleep(0.01) # Keep this short for responsive gyro integration
    # Ensure motors stop when loop finishes (either by target or timeout)
    # Assuming motor_control is the imported module
    motor_control.send_command(f"0.0 0.0 {0.0:.2f}\n")
    time.sleep(0.5) # Liten pause etter stopp

    # Log final state or reason for loop exit AFTER the loop
    end_time = time.time()
    duration = end_time - start_rotation_time
    final_error = target_angle_degrees - integrated_angle

    if abs(final_error) <= angle_tolerance:
        # Rotation completed successfully (within tolerance)
        # Log is handled by the calling function (main.py)
        skriv_logg(f"rotate_by_gyro completed. Target: {target_angle_degrees:.2f} , Reached: {integrated_angle:.2f} in {duration:.2f}s.")
    elif (end_time - start_rotation_time) >= rotation_timeout:
        # Rotation timed out
        skriv_logg(f"Warning: rotate_by_gyro timed out after {duration:.2f}s. Target: {target_angle_degrees:.2f} , Reached: {integrated_angle:.2f} , Remaining Error: {final_error:.2f} .")


# --- Heading Rotation Function (Absolute Heading using Compass) ---
def rotate_to_heading(heading_tracker, target_heading_degrees, timeout=20.0):

    global imu_initialized, bias_calibration_done
    # SJEKK/TILPASS: Antar at compass_module har en global flagg for initialisering
    # (compass_module.compass_initialized)
    if not imu_initialized or not bias_calibration_done or not compass_module.compass_initialized:
        skriv_logg("Cannot execute rotate_to_heading: IMU/Compass not initialized or calibrated.")
        # Antar motor_control er importert modul
        motor_control.send_command(f"0.0 0.0 {0.0:.2f}\n")
        return False # Return False ved feil

    # Sjekk om HeadingTracker objektet ble sendt inn
    if heading_tracker is None:
        skriv_logg("Error: rotate_to_heading requires a valid HeadingTracker instance.")
        motor_control.send_command(f"0.0 0.0 {0.0:.2f}\n")
        return False # Return False ved feil

    # Normalize target heading to be between 0 and 360
    target_heading_degrees = target_heading_degrees % 360.0
    if target_heading_degrees < 0:
        target_heading_degrees += 360

    # Sett rotation timeout
    rotation_timeout = 20.0 # seconds (Adjust as needed for large turns)
    start_rotation_time = time.time()

    # --- Initial lesing og klargj ring for D-term og loop ---
    heading_tracker.update()
    current_heading = heading_tracker.get_heading()
    korrigert_heading_steg1 = (current_heading + 45.0) % 360.0
    korrigert_heading = (korrigert_heading_steg1 + 2.0) % 360.0

    if current_heading == -1.0: # H ndter feil med f rste avlesning fra tracker
        skriv_logg("Error: Initial heading read failed in rotate_to_heading.")
        motor_control.send_command(f"0.0 0.0 {0.0:.2f}\n")
        return False # Avbryt funksjonen ved feil

    # Beregn f rste feil med wrap-around
    error = ((target_heading_degrees - korrigert_heading + 540.0) % 360.0) - 180.0

    # Initialiser variabler for PID-l kken
    previous_heading = korrigert_heading  # Lagre f rste heading som "forrige" (brukes ikke direkte i denne D-term metoden, men kan v re nyttig)
    last_time_pid = time.time() # Lagre f rste tidspunkt for dt beregning
    previous_error = error # Lagre den f rste feilen som "forrige" feil for D-term

    # ### LEGG TIL DENNE LINJEN HER F R WHILE-L KKEN STARTER: ###
    previous_smoothed_rate_of_change = 0.0
    skriv_logg(f"Executing initial rotation to {target_heading_degrees:.1f}  (Error: {error:.1f} )...")
    skriv_logg(f"Starting rotation (compass) to heading: {target_heading_degrees:.2f} ")

    # Bruk din definerte heading_tolerance fra toppen av filen (satt til 5.0)
    heading_tolerance = 5.0 # Henter konfigurasjonsverdi

    while (time.time() - start_rotation_time) < rotation_timeout:
        current_time_pid = time.time()
        dt = current_time_pid - last_time_pid
        last_time_pid = current_time_pid # Oppdater tidspunkt for neste iterasjon
        
        # --- HENT GJELDENDE FUSED HEADING ---
        heading_tracker.update() # <-- SJEKK/TILPASS: Kall update() her om HeadingTracker ikke har egen tr d
        # Hent den nylig oppdaterte fusede headingen for PID-feedback
        current_heading = heading_tracker.get_heading()
        korrigert_heading_steg1 = (current_heading + 45.0) % 360.0
        korrigert_heading = (korrigert_heading_steg1 + 2.0) % 360.0
        
        # H ndter feil med avlesning fra tracker inne i l kken
        if current_heading == -1.0:
            skriv_logg("Warning: Heading tracker returned -1.0 during rotation. Skipping PID update.")
            motor_control.send_command(f"0.0 0.0 {0.0:.2f}\n") # Stopp midlertidig ved sensorfeil
            time.sleep(0.1)
            previous_error = error # Behold forrige gyldige feil
            continue # G  til neste iterasjon

        # --- BEREGN FEIL ---
        error = ((target_heading_degrees - korrigert_heading + 540.0) % 360.0) - 180.0
        # --- DEBUG LOGG ---
        skriv_logg(f"PID Debug - Fused Heading: {korrigert_heading:.2f} , Error: {error:.2f} ")


        # Check if we are within the target tolerance
        if abs(error) <= heading_tolerance:
            # Rotation completed successfully (within tolerance)
            break # Exit loop if target reached

        # --- BEREGN D-TERM ---
        error_change = error - previous_error


        alpha_derivative = 0.3 # EMA smoothing factor (0.0 til 1.0). Lavere verdi = Mer glatting.
        # Beregn den r rate of change fra feilendring over tid (kan v re st yfull)
        raw_rate_of_change = (error_change / dt) if dt > 0 else 0.0
        # Beregn den glattede rate of change ved hjelp av EMA-formelen
        smoothed_rate_of_change = alpha_derivative * raw_rate_of_change + (1 - alpha_derivative) * previous_smoothed_rate_of_change
        # Lagre den glattede verdien for bruk i neste PID-l kke-iterasjon
        previous_smoothed_rate_of_change = smoothed_rate_of_change
        # Beregn derivat-termen basert p  GLATTET rate of change
        derivative_term = Kd_heading * smoothed_rate_of_change

        # Oppdater forrige feil for neste P-term og error_change beregning
        previous_error = error

        # --- BEREGN R  PID KOMMANDO P  INTERN SKALA --- (P-term (Kp * error) + D-term (Kd * rate_of_change)
        rotate_speed_command_internal_scale = (Kp_heading * error) + derivative_term # Bruker den n  oppdaterte derivative_term
        # --- Klemm R  PID KOMMANDO til intern skala [-base_speed_heading, base_speed_heading] ---
        base_speed_heading = 50 # Henter konfigurasjonsverdi (Sjekk din kode for hvor denne er definert/tilgjengelig)
        rotate_speed_command_internal_scale = max(min(rotate_speed_command_internal_scale, base_speed_heading), -base_speed_heading)
        # --- Konverter til GRADER/SEKUND ---
        MAX_ROBOT_ROTATION_DEGPS_AT_BASE_SPEED = 37.80 # Henter konfigurasjonsverdi (Sjekk din kode)
        rotation_command_dps = 0.0
        
        if base_speed_heading != 0: # Unng  deling p  null
            rotation_command_dps = (rotate_speed_command_internal_scale / base_speed_heading) * MAX_ROBOT_ROTATION_DEGPS_AT_BASE_SPEED
        # Handter minimum rotasjonshastighet i GRADER/SEKUND (bruker den beregnede MIN_ROBOT_ROTATION_DEGPS)
        MIN_ROBOT_ROTATION_DEGPS = (50.0 / base_speed_heading) * MAX_ROBOT_ROTATION_DEGPS_AT_BASE_SPEED # Sjekk om denne beregningen er korrekt eller om MIN_ROBOT_ROTATION_DEGPS er definert direkte

        if abs(error) > heading_tolerance and abs(rotation_command_dps) > 0.0 and abs(rotation_command_dps) < MIN_ROBOT_ROTATION_DEGPS:
            # Sett kommandoen til den minste deg/s verdien, med riktig fortegn som PID-utgangen (rotate_speed_command_internal_scale)
            rotation_command_dps = math.copysign(MIN_ROBOT_ROTATION_DEGPS, rotate_speed_command_internal_scale)

        # --- Send kommandoen til ESP32 ---
        command_to_send = f"{0.0:.2f} {0.0:.2f} {rotation_command_dps:.2f}\n" # <--- Send float deg/s
        motor_control.send_command(command_to_send) # Send kommandoen via motor_control modulen
        # --- Loop Delay ---
        time.sleep(0.05) # Setter PID loop frekvens til 20 Hz


    # --- Timeout skjedde eller m l n dd ---
    motor_control.send_command(f"0.0 0.0 {0.0:.2f}\n")
    time.sleep(0.5) # Liten pause etter stopp for   la roboten stanse fysisk

    # Les final heading ETTER at motor er stoppet for mest n yaktig sluttverdi
    heading_tracker.update() # Sjekk om HeadingTracker.update() skal kalles her for   f  fersk data
    current_heading_final = heading_tracker.get_heading()
    korrigert_heading_steg1_final = (current_heading_final + 45.0) % 360.0
    korrigert_heading_final = (korrigert_heading_steg1_final + 2.0) % 360.0

    # Beregn slutt-feil
    final_error = ((target_heading_degrees - korrigert_heading_final + 540.0) % 360.0) - 180.0

    # --- Logg sluttresultat ---
    end_time = time.time()
    duration = end_time - start_rotation_time

    if abs(final_error) <= heading_tolerance:
        # Logg den fullt korrigerte endelige headingen
        skriv_logg(f"Heading rotation to {target_heading_degrees:.2f} completed. Final heading: {korrigert_heading_final:.2f} in {duration:.2f}s.")
        return True

    elif (end_time - start_rotation_time) >= rotation_timeout:
        # S rg for   bruke korrigert_heading_final og final_error her ogs  i loggen
        skriv_logg(f"Heading rotation to {target_heading_degrees:.2f} timed out after {duration:.2f}s. Final heading: {korrigert_heading_final:.2f} , Remaining error: {final_error:.2f} .")
        return False

    else:
        # Usannsynlig   treffe denne hvis ikke brutt av brukeren tidligere
        # S rg for   bruke korrigert_heading_final og final_error her ogs  i loggen
        skriv_logg(f"Heading rotation to {target_heading_degrees:.2f} exited unexpectedly after {duration:.2f}s. Final heading: {korrigert_heading_final:.2f} , Remaining error: {final_error:.2f} .")
        return False