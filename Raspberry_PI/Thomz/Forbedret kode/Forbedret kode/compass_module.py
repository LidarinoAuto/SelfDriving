# Filename: compass_module.py
# Module for handling QMC5883L compass sensor data

# --- IMPORT NECESSARY LIBRARIES ---
import os
import smbus # For I2C communication
import time # For time functions (time.sleep())
import math # For mathematical functions (atan2, degrees)
from logging_utils import skriv_logg

# --- I2C Address and Register Addresses for QMC5883L ---
QMC5883L_ADDRESS = 0x0d # Standard I2C address for QMC5883L
# Moved bus initialization inside init_compass for better error handling
bus = None # Initialize bus as None initially

# Register Addresses for QMC5883L
QMC5883L_CTRL1 = 0x09 # Control Register 1 for sensor settings
QMC5883L_SET_RESET = 0x0B # Reset register
QMC5883L_DATA = 0x00 # Start register for X_L data (data begins here)

# --- Global Variables ---
compass_initialized = False # Flag to indicate successful initialization
offset_x = 0.0 # Global variable for X offset
offset_y = 0.0 # Global variable for Y offset

# --- INITIALIZATION ---
def init_compass():
    """
    Initializes the QMC5883L with the correct settings.
    Returns True if I2C communication for initialization succeeds, False otherwise.
    Offset loading errors do NOT cause initialization to fail critically.
    """
    global bus, compass_initialized, offset_x, offset_y # Declare global variables
    skriv_logg("Initializing QMC5883L...")
    try:
        bus = smbus.SMBus(1) # Standard I2C bus on Raspberry Pi (often 1)

        # Perform a soft reset by writing to the reset register
        bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_SET_RESET, 0x01)
        time.sleep(0.1) # Give the sensor some time after reset

        # Write to Control Register 1 to set mode and settings
        # Continuous mode, 50Hz ODR, 8G range, 256x oversampling
        # Check datasheet for exact bit meanings if needed
        bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_CTRL1, 0b00011101) # Example configuration
        time.sleep(0.1) # Wait for the sensor to start measurements

        skriv_logg("QMC5883L sensor initialized via I2C successfully.") # Justert loggtekst
        
        # --- LOAD CALIBRATION OFFSETS ---
        # Loading offsets is NOT critical for sensor initialization status
        try:
            if os.path.exists("kompas_offset.txt"):
                with open("kompas_offset.txt", "r") as f:
                    lines = f.readlines()
                    if len(lines) >= 2:
                        offset_x = float(lines[0].strip())
                        offset_y = float(lines[1].strip())
                        skriv_logg(f"Compass offsets loaded: x={offset_x:.2f}, y={offset_y:.2f}")
                    else:
                        skriv_logg("Warning: kompas_offset.txt found but does not contain enough lines.")
            else:
                skriv_logg("Warning: kompas_offset.txt not found. Running without compass calibration offsets.")
        except Exception as e:
                skriv_logg(f"Warning: Error loading compass offsets: {e}")
        # --- END LOAD CALIBRATION OFFSETS ---

        compass_initialized = True # <-- SETT TIL TRUE HER VED SUKSESS MED I2C
        return True # <--- RETURNER TRUE HVIS I2C INITIALISERING LYKTES
    
    except FileNotFoundError:
        skriv_logg("Error: I2C bus not found for compass. Make sure I2C is enabled.")
        compass_initialized = False # Sett til False ved feil
        bus = None # Sett bus til None ved feil
        return False # <--- RETURNER FALSE VED KRITISK FEIL
    except Exception as e:
        skriv_logg(f"Error during QMC5883L I2C initialization: {e}") # Justert loggtekst
        compass_initialized = False # Sett til False ved feil
        bus = None # Sett bus til None ved feil
        return False # <--- RETURNER FALSE VED KRITISK FEIL


# --- READING COMPASS DATA ---
def read_raw_xy():
    #"""Reads raw X and Y data from QMC5883L sensor."""
    global bus, QMC5883L_ADDRESS
    # Legg til sjekk for compass_initialized her ogs 
    if not compass_initialized or bus is None:
        return 0, 0 # Return 0s if not initialized or bus is not available

    try:
        # Ensure the sensor is in continuous measurement mode if not already
        # This might be redundant if init_compass already sets this,
        # but safe to ensure for raw reads.
        # We saw 0x1D (0b00011101) in working code, let's ensure that mode
        # bus.write_byte_data(QMC5883L_ADDRESS, 0x09, 0x1D) # Dette er CTRL1, modus er 0x0B = 0x01? Sjekk datasheet
        # bus.write_byte_data(QMC5883L_ADDRESS, 0x0B, 0x01) # Sjekk datasheet for dette!
        # Ok, den forrige koden hadde 0x09, 0b00011101 som CTRL1. Det er nok riktig.
        # Fjern ekstra skriving til register i read_raw_xy hvis init_compass gj r det
        # Dette kan forstyrre lesingen.

        # Les 6 bytes av data fra QMC5883L, starter fra register 0x00 (Data Register)
        data = bus.read_i2c_block_data(QMC5883L_ADDRESS, QMC5883L_DATA, 6)

        # Konverter raw bytes til signed 16-bit integers (Little-Endian)
        x_raw = (data[1] << 8) | data[0]
        y_raw = (data[3] << 8) | data[2]
        # z_raw = (data[5] << 8) | data[4] # Z data is typically not used for 2D heading

        # Apply two's complement conversion for signed values
        # QMC5883L raw data are signed 16-bit values. This conversion handles the sign bit.
        x = x_raw - 65536 if x_raw > 32767 else x_raw
        y = y_raw - 65536 if y_raw > 32767 else y_raw
        # z = z_raw - 65536 if z_raw > 32767 else z_raw # Not needed for 2D heading

        return x, y
    except Exception as e:
        # Avoid crashing the calibration tool on read error
        # print(f"Error reading raw compass data: {e}") # Dette er for calibration tool
        return 0, 0 # Return zeros on error


def read_compass_data(): # <--- DETTE ER FUNKSJONEN MAIN.PY TRENGER!
    """
    Reads raw data from QMC5883L (X, Y), applies calibration offset and upside-down compensation,
    and calculates heading in degrees.
    Returns the heading in degrees (0-359.9), or -1.0 on error or if not initialized.
    """
    # Declare global variables, including offsets loaded in init_compass
    global compass_initialized, offset_x, offset_y # Bruk de globale offsetene

    # Check if the compass sensor has been successfully initialized
    if not compass_initialized or bus is None: # Sjekk ogs  om bus er satt
        # skriv_logg("Warning: Attempted to read compass before initialization.") # Optional debug
        return -1.0 # Return -1.0 if not initialized or bus is not available

    try:
        # Read raw X and Y data using the helper function
        x_raw, y_raw = read_raw_xy() # Kall hjelpefunksjonen

        # Hvis raw data var 0 pga feil i read_raw_xy, returner feil
        # En enkel sjekk, kan forbedres
        if x_raw == 0 and y_raw == 0:
            # skriv_logg("Warning: Raw compass data was zero.") # Kan spamme loggen
            return -1.0

        # --- APPLY CALIBRATION OFFSETS (Hard-Iron Compensation) ---
        # Subtract the loaded hard-iron offsets
        compensated_x = x_raw - offset_x
        compensated_y = y_raw - offset_y
        # --- END APPLY CALIBRATION OFFSETS ---


        # --- COMPENSATE FOR UPSIDE-DOWN MOUNTING (if sensor is mounted upside down) ---
        # Viktig: Sjekk at denne kompensasjonen (y, -x) stemmer for DIN montering!
        # Standard atan2(y, x) gir 0 mot +x ( st). Kompass 0 er mot +y (Nord).
        # Hvis din sensor gir +Y mot Nord og +X mot  st, s  skal du IKKE bruke (y, -x).
        # Da skal du sannsynligvis bruke (x, y) eller (y, x) eller lignende.
        # La oss holde (y, -x) basert p tidligere logger hvor det s  ut til   virke for heading.
        # Det er en vanlig kompensasjon for en standardorientert sensor snudd 180 grader rundt X-aksen.
        final_x_for_atan2 = compensated_y  # Offset-korrigert Y-akse (som peker N/S)
        final_y_for_atan2 = -compensated_x # Negativ offset-korrigert X-akse (som peker  /W)
        # --- END COMPENSATE FOR UPSIDE-DOWN MOUNTING ---


        # Calculate heading in radians using atan2(final_y_for_atan2, final_x_for_atan2)
        heading_rad = math.atan2(final_y_for_atan2, final_x_for_atan2)


        # Convert heading from radians to degrees
        heading_deg = math.degrees(heading_rad)

        # Normalize heading to be between 0 and 360 degrees
        # atan2 returns values from -180 to +180 degrees. Vi vil ha 0-360.
        if heading_deg < 0:
            heading_deg += 360

        # --- DEBUG LOG ---
        skriv_logg(f"Compass Debug Heading: {heading_deg:.2f}")
        # --- END DEBUG LOG ---

        # Return the calculated heading in degrees
        return heading_deg # <-- Returnerer den beregnede headingen

    except Exception as e:
        # Handle potential errors during I2C communication or calculation
        # skriv_logg(f"Error reading QMC5883L: {e}") # Optional: Uncomment for debugging errors
        # Return -1.0 on error (can be handled in the main program)
        return -1.0 # Return -1.0 on error


def get_compass_direction(deg):
    """
    Converts degrees to a cardinal direction string (N, NE, E, etc.).
    Returns a direction based on input degrees.
    """
    # List of the 16 main directions for precise navigation
    dirs = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW']

    # Handle any abnormal degrees outside 0-360 range (though read_compass normalizes)
    if deg is None or deg < 0 or deg > 360:
        return "Unknown"

    # Calculate index based on degrees. Add 11.25 to center around North (0/360 degrees)
    # 360 / 16 = 22.5 degrees per sector
    index = int((deg + 11.25) % 360 // 22.5)

    # Return the calculated direction
    return dirs[index]

# Note: No if __name__ == "__main__": block here. Code is started from main.py.