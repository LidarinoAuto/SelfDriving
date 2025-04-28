# -*- coding: utf-8 -*-
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
compass_initialized = False # <-- NY VARIABEL: Flag to indicate successful initialization
offset_x = 0.0 # <-- Add this global variable for X offset
offset_y = 0.0 # <-- Add this global variable for Y offset

# --- INITIALIZATION ---
def init_compass():
    """Initializes the QMC5883L with the correct settings."""
    global bus, compass_initialized # Declare global variables
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

        skriv_logg("QMC5883L initialized successfully.")
        
        # --- LOAD CALIBRATION OFFSETS ---  # <--- Add this block
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
                skriv_logg("kompas_offset.txt not found. Running without compass calibration offsets.")
        except Exception as e:
                skriv_logg(f"Error loading compass offsets: {e}")
        
        # --- END LOAD CALIBRATION OFFSETS ---
        
        compass_initialized = True # <-- SETT TIL TRUE VED SUKSESS
    except FileNotFoundError:
        skriv_logg("Error: I2C bus not found for compass. Make sure I2C is enabled.")
        compass_initialized = False # Sett til False ved feil
        bus = None # Sett bus til None ved feil
    except Exception as e:
        skriv_logg(f"Error during QMC5883L initialization: {e}")
        compass_initialized = False # Sett til False ved feil
        bus = None # Sett bus til None ved feil


# --- READING COMPASS DATA ---
def read_compass():
    """
    Reads raw data from QMC5883L (X, Y, Z), applies calibration offset and upside-down compensation,
    and calculates heading in degrees.
    Returns the heading in degrees (0-359), or -1.0 on error or if not initialized.
    """
    # Declare global variables, including offsets loaded in init_compass
    global bus, compass_initialized, offset_x, offset_y

    # Check if the compass sensor has been successfully initialized
    if not compass_initialized or bus is None:
        # skriv_logg("Warning: Attempted to read compass before initialization.") # Optional debug
        return -1.0 # Return -1.0 if not initialized or bus is not available

    try:
        # Read 6 bytes of data from QMC5883L, starting from register 0x00 (Data Register)
        # Data comes in the order X_L, X_H, Y_L, Y_H, Z_L, Z_H (Little-Endian)
        data = bus.read_i2c_block_data(QMC5883L_ADDRESS, QMC5883L_DATA, 6)

        # Reconstruct 16-bit signed integer values from the 2 bytes for each axis (Little-Endian)
        x_raw = (data[1] << 8) | data[0]
        y_raw = (data[3] << 8) | data[2]
        z_raw = (data[5] << 8) | data[4]

        # Convert from unsigned 16-bit values to signed (two's complement)
        # QMC5883L raw data are signed 16-bit values. This conversion handles the sign bit.
        x = x_raw - 65536 if x_raw > 32767 else x_raw
        y = y_raw - 65536 if y_raw > 32767 else y_raw
        # z = z_raw - 65536 if z_raw > 32767 else z_raw # Z data is typically not used for 2D heading


        # --- APPLY CALIBRATION OFFSETS (Hard-Iron Compensation) ---
        # Subtract the loaded hard-iron offsets (calculated by calibrate_compass_tool)
        # from the raw signed X and Y values. This shifts the magnetic center to (0,0).
        compensated_x_raw = x - offset_x
        compensated_y_raw = y - offset_y
        # --- END APPLY CALIBRATION OFFSETS ---


        # --- COMPENSATE FOR UPSIDE-DOWN MOUNTING (if sensor is mounted upside down) ---
        # Apply the upside-down compensation to the offset-corrected values.
        # The standard heading calculation uses atan2(y, x) where +x is East and +y is North.
        # Adjust the offset-corrected values (compensated_x_raw, compensated_y_raw)
        # based on how YOUR compass is physically mounted relative to the robot's desired North axis.
        # The mapping (y, -x) is a common compensation for Z-axis flip, assuming raw Y points
        # towards magnetic North/South and raw X towards magnetic East/West after offset.
        # IMPORTANT: Verify if this mapping (y, -x) matches your physical mounting and desired heading directions!
        final_x_for_atan2 = compensated_y_raw # Use offset-corrected y as the X component for atan2 (maps to North/South axis)
        final_y_for_atan2 = -compensated_x_raw # Use negative offset-corrected x as the Y component for atan2 (maps to East/West axis)
        # --- END COMPENSATE FOR UPSIDE-DOWN MOUNTING ---


        # Calculate heading in radians using atan2(final_y_for_atan2, final_x_for_atan2)
        # atan2(y, x) calculates the angle between the positive X-axis and the point (x, y).
        # Our final_x_for_atan2 represents the X-axis component (North/South),
        # and final_y_for_atan2 represents the Y-axis component (East/West) relative to Magnetic North.
        # Note: atan2(y, x) maps (+x, +y) -> 0 to +90, (-x, +y) -> +90 to +180, (-x, -y) -> -180 to -90, (+x, -y) -> -90 to 0
        heading_rad = math.atan2(final_y_for_atan2, final_x_for_atan2) # <-- KUN EN BEREGNING AV heading_rad HER


        # Convert heading from radians to degrees
        heading_deg = math.degrees(heading_rad)

        # Normalize heading to be between 0 and 360 degrees
        # atan2 returns values from -180 to +180 degrees. We want 0-360.
        if heading_deg < 0:
            heading_deg += 360

        # --- DEBUG LOG ---
        # Log the calculated heading for debugging purposes
        skriv_logg(f"Compass Debug Heading: {heading_deg:.2f}") # <-- Loggen skal v re her
        # --- END DEBUG LOG ---

        # Return the calculated heading in degrees
        # Note: The original code returned heading_deg and raw data (x, y, z).
        # main.py currently only uses the heading. Returning only heading_deg.
        return heading_deg # <-- Return statement skal v re her

    except Exception as e:
        # Handle potential errors during I2C communication or calculation
        # skriv_logg(f"Error reading QMC5883L: {e}") # Optional: Uncomment for debugging errors
        # Return -1.0 on error (can be handled in the main program)
        return -1.0 # Return -1.0 on error
    
def read_raw_xy():
    #"""Reads raw X and Y data from QMC5883L sensor."""
    global bus, QMC5883L_ADDRESS
    try:
        # Ensure the sensor is in continuous measurement mode if not already
        # This might be redundant if init_compass already sets this,
        # but safe to ensure for raw reads.
        # We saw 0x1D (0b00011101) in working code, let's ensure that mode
        bus.write_byte_data(QMC5883L_ADDRESS, 0x09, 0x1D) # QMC5883L_CTRL1 = 0x09, Mode=0x1D

        data = bus.read_i2c_block_data(QMC5883L_ADDRESS, 0x00, 6) # Start reading from 0x00

        # Convert the raw bytes to signed 16-bit integers
        x_raw = (data[1] << 8) | data[0]
        y_raw = (data[3] << 8) | data[2]
        z_raw = (data[5] << 8) | data[4] # Read Z too, though we only need X/Y for 2D heading

        # Apply two's complement conversion for signed values
        x = x_raw - 65536 if x_raw > 32767 else x_raw
        y = y_raw - 65536 if y_raw > 32767 else y_raw
        # z = z_raw - 65536 if z_raw > 32767 else z_raw # Not needed for 2D heading

        return x, y
    except Exception as e:
        # Avoid crashing the calibration tool on read error
        print(f"Error reading raw compass data: {e}")
        return 0, 0 # Return zeros on error


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