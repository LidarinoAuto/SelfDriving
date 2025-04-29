import smbus    # For I2C communication
import time
import math
from logging_utils import skriv_logg
import motor_control # Import motor_control module
import compass_module # Import compass_module

# --- Configuration ---
# IMPORTANT: Verify these settings for YOUR specific IMU chip!
# Common for MPU-6050:
IMU_ADDRESS = 0x68      # I2C address (0x68 or 0x69) - Check with i2cdetect -y 1
PWR_MGMT_1 = 0x6B       # Power Management Register 1
GYRO_ZOUT_H = 0x47      # High byte of Z-axis gyro data

# Gyroscope scale factor (adjust based on your IMU configuration)
# This scales the raw 16-bit value to degrees/second.
# For MPU-6050, depends on GYRO_CONFIG register setting:
# 0x00 (+/- 250 deg/sec) -> 131.0 LSB/deg/sec -> Scale: 1/131.0 = 0.007633
# 0x08 (+/- 500 deg/sec) -> 65.5 LSB/deg/sec  -> Scale: 1/65.5 = 0.015267
# 0x10 (+/- 1000 deg/sec)-> 32.8 LSB/deg/sec  -> Scale: 1/32.8 = 0.03049
# 0x18 (+/- 2000 deg/sec)-> 16.4 LSB/deg/sec  -> Scale: 1/16.4 = 0.06097
# ASSUMING +/- 250 deg/sec range as a common default
GYRO_SCALE_FACTOR = 1.0 / 131.0 # Or adjust based on your GYRO_CONFIG

# --- Global Variables ---
bus = None
integrated_angle = 0.0    # Integrated angle for current rotation (used by rotate_by_gyro)
last_time_gyro = 0.0         # Time of the last IMU reading for integration (for rotate_by_gyro)
gyro_bias_z = 0.0       # Calculated Z-axis gyro bias
bias_calibration_done = False # Flag
imu_initialized = False # Added initialization flag

# --- Initialization ---
# Updated to set global flag and return status
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

        # Wake up IMU (command depends on chip, MPU-6050 uses PWR_MGMT_1 = 0)
        # IMPORTANT: Check if this command is correct for your IMU!
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
# IMPORTANT: Adjust register addresses and byte logic for YOUR specific IMU chip!
def read_raw_gyro_z():
    """Reads raw Z-axis gyroscope data from the IMU."""
    global bus
    if bus is None:
        # skriv_logg("Warning: Attempted to read raw gyro before bus initialized.") # Optional debug log
        return 0 # Return 0 if bus is not initialized or error occurred earlier

    try:
        # Read high and low byte for Z-axis gyro
        # IMPORTANT: Verify these register addresses for YOUR IMU! (e.g. 0x47 for MPU-6050)
        gyro_z_h = bus.read_byte_data(IMU_ADDRESS, GYRO_ZOUT_H)
        gyro_z_l = bus.read_byte_data(IMU_ADDRESS, GYRO_ZOUT_H + 1)

        # Combine bytes to a signed 16-bit integer
        raw_value = (gyro_z_h << 8) | gyro_z_l
        if raw_value > 32767: # Handle negative numbers (2's complement)
            raw_value -= 65536

        return raw_value
    
    except Exception:
        # Return 0 on error - this might cause inaccurate integration if reads fail mid-turn
        # skriv_logg(f"Warning: Error reading raw gyro data: {e}") # Optional debug skriv_logg - kan spamme loggen
        return 0
    
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
    # Negate Z-axis velocity if IMU is mounted upside-down relative to convention
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
        motor_control.send_command("0 0 0\n") # Ensure motors are stopped
        return # Exit function if IMU is not ready
    
    # Reset integrated angle BEFORE starting THIS rotation
    # This is crucial: each relative rotation starts from 0 integrated angle for THAT turn
    integrated_angle = 0.0
    last_time_gyro = time.time() # Get start time for this rotation

    skriv_logg(f"Starting rotation (gyro): {target_angle_degrees:.2f} ") # Added skriv_logg with degree symbol

    # --- Rotation Control Parameters (Adjust these for tuning) ---
    # Simple proportional control: Speed = Kp_speed * Error
    Kp_speed = 0.4 # Proportional gain for speed control (Lower = slower, Higher = faster turn)
    angle_tolerance = 1.0 # Stop when within +/- 1.0 degrees of target (increased slightly for stability)
    base_speed = 18 # Max/Base motor speed for rotation (kept at 18)
    min_rotate_speed = 5 # Minimum motor speed to prevent stalling near target (kept at 5)

    # Set a timeout for rotation to prevent infinite loops if IMU fails mid-turn
    rotation_timeout = 10.0 # seconds (Adjust as needed)
    start_rotation_time = time.time()
    
    # Rotation control loop based on integrated angle
    # Continue as long as error is outside tolerance AND timeout not reached
    # The condition checks if the *absolute* difference is greater than tolerance
    while abs(target_angle_degrees - integrated_angle) > angle_tolerance and (time.time() - start_rotation_time) < rotation_timeout:
        current_time = time.time()
        # Calculate time difference since last reading
        # Use a small minimum dt to avoid division by zero issues if loop is very fast
        dt = max(current_time - last_time_gyro, 0.0001)
        last_time_gyro = current_time # Update last time for the next iteration

        # Read compensated angular velocity (degrees/sec)
        current_velocity = read_imu_data()

        # Integrate velocity over time to get current angle
        integrated_angle += current_velocity * dt

        # Calculate error (remaining angle needed)
        error = target_angle_degrees - integrated_angle

        # Calculate motor speed command based on proportional control
        # The sign of 'error' gives the direction. Need negative here? Depends on motor control and gyro sign.
        # Let's assume this sign works for rotate_by_gyro based on previous behavior
        rotate_speed_command = -Kp_speed * error # <--- BEHOLD MINUS HER for rotate_by_gyro

        # Apply speed limits and minimum speed based on the calculated command's magnitude.
        # Use absolute value for clamping, then reapply the original sign.
        abs_speed_command = abs(rotate_speed_command)
        
        # Clamp between min_rotate_speed and base_speed
        if abs_speed_command < min_rotate_speed:
            # If below min, use min speed, but keep original sign
            clamped_abs_speed = min_rotate_speed
        elif abs_speed_command > base_speed:
            # If above max, use base speed
            clamped_abs_speed = base_speed
        else:
            # If within range, use calculated speed magnitude
            clamped_abs_speed = abs_speed_command
            
        # Reapply the original sign and convert to integer for motor command
        # math.copysign(magnitude, sign_value) returns magnitude with the sign of sign_value
        direction_sign = 1 if error >= 0 else -1 # Bruk feil-fortegnet for retningen

        # Apply the correct direction sign to the clamped absolute speed magnitude
        send_speed = int(clamped_abs_speed * direction_sign)

        # Debug logg (Optional, can make log very long - KEEP COMMENTED OUT for clean log)
        #skriv_logg(f"Integ: {integrated_angle:.2f}  / Target: {target_angle_degrees:.2f}  (Remaining: {error:.2f} ), Speed: {send_speed}")

        motor_control.send_command(f"0 0 {send_speed}\n")

        time.sleep(0.01) # Keep this short for responsive gyro integration

    # Ensure motors stop when loop finishes (either by target or timeout)
    motor_control.send_command("0 0 0\n")
    time.sleep(0.5) # Liten pause etter stopp

    # Optional: Log final state or reason for loop exit AFTER the loop
    end_time = time.time()
    duration = end_time - start_rotation_time
    final_error = target_angle_degrees - integrated_angle

    if abs(final_error) <= angle_tolerance:
        # Rotation completed successfully (within tolerance)
        # Log is handled by the calling function (main.py)
        pass
    elif (end_time - start_rotation_time) >= rotation_timeout:
        # Rotation timed out
        skriv_logg(f"Warning: rotate_by_gyro timed out after {duration:.2f}s. Target: {target_angle_degrees:.2f} , Reached: {integrated_angle:.2f} , Remaining Error: {final_error:.2f} .")
        
# --- Heading Rotation Function (Absolute Heading using Compass) ---
def rotate_to_heading(target_heading_degrees, heading_tracker):
    """
    Rotates the robot to a specific absolute heading (in degrees, 0-359)
    using Compass feedback. North is 0 degrees.
    Requires IMU and Compass to be initialized.
    Uses proportional and Derivative control based on heading error and rate of change.
    More accurate for navigating to fixed directions than rotate_by_gyro.
    Requires a HeadingTracker object instance to be passed in.
    """
    global imu_initialized, bias_calibration_done
    # Trenger ogs    sjekke om compass_module er initialisert, da HeadingTracker.update() bruker den.
    # Vi sjekker dette indirekte ved   sjekke om HeadingTracker-objektet ble opprettet i main,
    # men en eksplisitt sjekk her ogs  er sikrere.
    if not imu_initialized or not bias_calibration_done or not compass_module.compass_initialized:
        skriv_logg("Cannot execute rotate_to_heading: IMU/Compass not initialized or calibrated.")
        motor_control.send_command("0 0 0\n")
        return

    # Sjekk om HeadingTracker objektet ble sendt inn
    if heading_tracker is None:
        skriv_logg("Error: rotate_to_heading requires a valid HeadingTracker instance.")
        motor_control.send_command("0 0 0\n")
        return
    
    # Normalize target heading to be between 0 and 360
    target_heading_degrees = target_heading_degrees % 360

    skriv_logg(f"Starting rotation (compass) to heading: {target_heading_degrees:.2f} ")

    # --- Rotation Control Parameters (Adjust these for tuning) ---
    # PD control: Speed = Kp * Error + Kd * dError/dt
    Kp_heading = 0.1 # Proportional gain for heading control (Adjust this!)
    Kd_heading = 0.3 # Derivative gain for damping oscillations (Adjust this!)
    heading_tolerance = 5.0 # Stop when within +/- 5.0 degrees of target heading (Adjust!)
    base_speed_heading = 50 # Max/Base motor speed for heading rotation (Adjust if needed)
    min_rotate_speed_heading = 8 # Minimum motor speed for heading rotation (Adjust if needed)

    # Set a timeout for rotation
    rotation_timeout = 20.0 # seconds (Adjust as needed for large turns)
    start_rotation_time = time.time()


    # --- FORSTE AVLESNING OG INITIALISERING FOR D-TERM ---
    # Les av heading f rste gang for   sjekke initialtilstand og sette opp D-term variabler
    # Bruk HeadingTracker objektet som ble sendt inn
    heading_tracker.update() # Oppdater fusjonen med nye sensoravlesninger
    current_heading = heading_tracker.get_heading() # Hent fuset heading
    
    if current_heading == -1.0: # H ndter feil med f rste avlesning fra tracker (skal normalt ikke skje hvis init lyktes)
        skriv_logg("Error: Initial heading read failed in rotate_to_heading.")
        motor_control.send_command("0 0 0\n") # Stopp motor
        return # Avbryt funksjonen

    error = ((target_heading_degrees - current_heading + 540.0) % 360.0) - 180.0

    # Initialiser variabler for D-term beregning inne i loopen
    previous_heading = current_heading  # Lagre f rste heading som "forrige"
    last_time_pid = time.time() # Lagre f rste tidspunkt for dt beregning
    previous_error = error # Lagre den forste feilen som "forrige" feil for D-term


    # Rotation control loop based on heading error (from compass)
    while (time.time() - start_rotation_time) < rotation_timeout:
        # --- UPDATE AND GET CURRENT FUSED HEADING (INNE I L KKEN) ---
        # Disse kalles N  for HVER gang l kken kj rer, for oppdaterte verdier
        heading_tracker.update() # <-- VIKTIG! Oppdater trackeren i hver loop-iterasjon

        # Hent den nylig oppdaterte fusede headingen for PID-feedback
        current_heading = heading_tracker.get_heading()

        # HeadingTracker get_heading() skal normalt ikke returnere -1.0 hvis initialisering lyktes,
        # men legger til en sjekk for sikkerhet (hvis update feilet feks)
        if current_heading == -1.0: # H ndter feil inne i l kken
            skriv_logg("Warning: Heading tracker returned -1.0 during rotation. Skipping PID update.")
            motor_control.send_command("0 0 0\n") # Stopp motorene midlertidig ved sensorfeil
            time.sleep(0.1) # Hold denne p 0.1 for a unng spamme loopen
            # Tilbakestill tid og forrige heading for   unng  store sprett etter en feilavlesning
            last_time_pid = time.time()
            # Pr v   f  en ny heading for previous_heading hvis mulig, ellers behold den gamle
            temp_heading = heading_tracker.get_heading()
            previous_heading = temp_heading if temp_heading != -1.0 else previous_heading
            continue # G  til neste iterasjon
            
            
        # --- BEREGN P-TERM OG FEIL ---
        # Calculate heading error (difference between target and current heading)
        # Bruk smidig wrap-around for   finne korteste vei (error i [-180, 180])
        error = ((target_heading_degrees - current_heading + 540.0) % 360.0) - 180.0


        # --- DEBUG LOGG FOR FUSED HEADING OG FEIL ---
        # Veldig viktig for a se hva PID-en reagerer p
        # Kan kommenteres ut nar tuning er ferdig
        skriv_logg(f"PID Debug - Fused Heading: {current_heading:.2f} , Error: {error:.2f} ")


        # Check if we are within the target tolerance
        if abs(error) <= heading_tolerance:
            # Rotation completed successfully (within tolerance)
            # Den endelige loggen skrives etter loopen er ferdig.
            break # Exit loop if target reached


        # --- BEREGN D-TERM ---
        current_time_pid = time.time()
        dt = current_time_pid - last_time_pid
        last_time_pid = current_time_pid # Oppdater tidspunkt for neste iterasjon

        # Beregn endring i feilen siden sist iterasjon
        error_change = error - previous_error

        # Beregn derivat-termen basert p endring i feil (unng  div by zero hvis dt er 0)
        derivative_term = (Kd_heading * error_change / dt) if dt > 0 else 0.0


        # --- BEREGN TOTAL PID KOMMANDO ---
        # P-term (Kp * error) + D-term (Kd * rate_of_change)
        # Basert p siste logg: Positiv kommando = CW sving.
        # Positiv feil (trenger CW sving) -> Positiv kommando.
        # Negativ feil (trenger CCW sving) -> Negativ kommando.
        # Kp * error gir riktig fortegn.
        rotate_speed_command = (Kp_heading * error) + derivative_term # <--- P + D KOMMANDO (UTEN MINUS FORAN Kp)


        # Apply speed limits and minimum speed based on the calculated command's magnitude.
        # Use absolute value for clamping, then reapply the original sign.
        abs_speed_command = abs(rotate_speed_command)
        
        # Clamp between min_rotate_speed_heading and base_speed_heading
        if abs_speed_command < min_rotate_speed_heading and abs(error) > heading_tolerance:
            # Hvis under min speed, men vi ER IKKE i m l, bruk minimum hastighet
            clamped_abs_speed = min_rotate_speed_heading
        elif abs_speed_command > base_speed_heading:
            # Hvis over maks, bruk base speed
            clamped_abs_speed = base_speed_heading
        else:
            # Hvis innenfor omr de eller allerede i m l (error <= tolerance), bruk beregnet speed magnitude
            # abs(error) <= heading_tolerance sjekkes over, s  denne gj r ingenting om den ikke er brekket ut
            clamped_abs_speed = abs_speed_command
            
        # Reapply the original sign and convert to integer for motor command
        # math.copysign(magnitude, sign_value) returns magnitude with the sign of sign_value
        send_speed = int(math.copysign(clamped_abs_speed, rotate_speed_command))

        previous_error = error        

        # Send motor kommando
        motor_control.send_command(f"0 0 {send_speed}\n")

        time.sleep(0.2)  


    # Ensure motors stop when loop finishes (either by target or timeout)
    motor_control.send_command("0 0 0\n") # Stopp motor etter l kken
    time.sleep(0.5) # Liten pause etter stopp

    # Log final state or reason for loop exit
    end_time = time.time()
    duration = end_time - start_rotation_time
    # Les final heading ETTER at motor er stoppet for mest n yaktig sluttverdi
    # Bruk HeadingTracker objektet
    heading_tracker.update() # En siste oppdatering
    current_heading_final = heading_tracker.get_heading()

    # Beregn slutt-feil
    final_error = ((target_heading_degrees - current_heading_final + 540.0) % 360.0) - 180.0


    if abs(final_error) <= heading_tolerance: # Sjekk igjen med den helt siste avlesningen
        skriv_logg(f"Heading rotation to {target_heading_degrees:.2f}  completed. Final heading: {current_heading_final:.2f}  in {duration:.2f}s.")
    elif (end_time - start_rotation_time) >= rotation_timeout:
        skriv_logg(f"Heading rotation to {target_heading_degrees:.2f}  timed out after {duration:.2f}s. Final heading: {current_heading_final:.2f} , Remaining error: {final_error:.2f} .")
    else:
        # Usannsynlig   treffe denne hvis ikke brutt av brukeren tidligere
        skriv_logg(f"Heading rotation to {target_heading_degrees:.2f}  exited unexpectedly after {duration:.2f}s. Final heading: {current_heading_final:.2f} , Remaining error: {final_error:.2f} .")