# -*- coding: utf-8 -*-
# Filename: compass_module.py
# Module for handling QMC5883L compass sensor data

# --- IMPORT NECESSARY LIBRARIES ---
import smbus # For I2C communication
import time # For time functions (time.sleep())
import math # For mathematical functions (atan2, degrees)

# --- I2C Address and Register Addresses for QMC5883L ---
QMC5883L_ADDRESS = 0x0d # Standard I2C address for QMC5883L
bus = smbus.SMBus(1) # Standard I2C bus on Raspberry Pi (often 1)

# Register Addresses for QMC5883L
QMC5883L_CTRL1 = 0x09 # Control Register 1 for sensor settings
QMC5883L_SET_RESET = 0x0B # Reset register
QMC5883L_DATA = 0x00 # Start register for X_L data (data begins here)


# --- INITIALIZATION ---
def init_compass():
    """Initializes the QMC5883L with the correct settings."""
    print("Initializing QMC5883L...")
    try:
        # Perform a soft reset by writing to the reset register
        bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_SET_RESET, 0x01)
        time.sleep(0.1) # Give the sensor some time after reset

        # Write to Control Register 1 to set mode and settings
        # Continuous mode, 50Hz ODR, 8G range, 256x oversampling
        # Check datasheet for exact bit meanings if needed
        bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_CTRL1, 0b00011101) # Example configuration
        time.sleep(0.1) # Wait for the sensor to start measurements

        print("QMC5883L initialized successfully.")
    except Exception as e:
        print(f"Error during QMC5883L initialization: {e}")
        # Consider handling the error further, e.g., exit or retry


# --- READING COMPASS DATA ---
def read_compass():
    """
    Reads raw data from QMC5883L (X, Y, Z) and calculates heading in degrees.
    Returns the heading in degrees.
    """
    try:
        # Read 6 bytes of data from QMC5883L, starting from register 0x00
        # Data comes in the order X_L, X_H, Y_L, Y_H, Z_L, Z_H (Little-Endian)
        data = bus.read_i2c_block_data(QMC5883L_ADDRESS, QMC5883L_DATA, 6)

        # Reconstruct 16-bit values from the 2 bytes (Little-Endian)
        x_raw = (data[1] << 8) | data[0]
        y_raw = (data[3] << 8) | data[2]
        z_raw = (data[5] << 8) | data[4]

        # Convert from unsigned 16-bit values to signed (two's complement)
        # QMC5883L raw data are signed 16-bit values
        x = x_raw - 65536 if x_raw > 32767 else x_raw
        y = y_raw - 65536 if y_raw > 32767 else y_raw
        z = z_raw - 65536 if z_raw > 32767 else z_raw

        # --- COMPENSATE FOR UPSIDE-DOWN MOUNTING ---
        # The standard heading calculation uses atan2(y, x).
        # With the sensor upside down, the raw x, y, z readings need to be
        # remapped to the robot's forward/sideways axes relative to the magnetic field.
        # A common compensation for Z-axis flip is swapping and/or negating X and Y.
        # Let's try using -x as the new Y component and y as the new X component for atan2.
        # This maps the sensor's raw X (with flipped sign) to the magnetic East/West
        # and raw Y to the magnetic North/South, assuming a typical flip.
        compensated_x = y # Use raw signed y as the X component for atan2 (North/South)
        compensated_y = -x # Use negative raw signed x as the Y component for atan2 (East/West)

        # --- CUSTOM CALIBRATION OFFSET (Optional) ---
        # If needed, add hard-iron calibration offset here by subtracting constants
        # compensated_x = compensated_x - x_offset
        # compensated_y = compensated_y - y_offset


        # Calculate heading in radians using atan2(compensated_y, compensated_x)
        # atan2(y, x) gives the angle from the positive X-axis (North after compensation)
        heading_rad = math.atan2(compensated_y, compensated_x) # Use compensated values


        # Convert from radians to degrees
        heading_deg = math.degrees(heading_rad)

        # Normalize heading to be between 0 and 360 degrees
        # atan2 returns values from -180 to +180 degrees
        if heading_deg < 0:
             heading_deg += 360

        # Note: The original code returned heading_deg and raw data (x, y, z).
        # main.py currently only uses the heading. Returning only heading_deg.
        return heading_deg # Return only the heading

    except Exception as e:
        # print(f"Error reading QMC5883L: {e}") # Optional: Uncomment for debugging errors
        # Return -1 on error (can be handled in the main program)
        return -1


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