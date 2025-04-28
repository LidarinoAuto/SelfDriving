# calibrate_compass_tool.py
import time
import math # Kan trenges senere for visning/validering, men ikke strengt nødvendig for bare min/max
import sys  # For å avslutte pent
from logging_utils import skriv_logg # Antar du har denne, ellers bruk print
import compass_module # Import the compass module to use its functions

def main():
    skriv_logg("Starting compass calibration tool...")
    skriv_logg("Initializing compass sensor...")

    try:
        compass_module.init_compass() # Use the init function from your compass_module
        skriv_logg("Compass initialized successfully.")
    except Exception as e:
        skriv_logg(f"Error initializing compass: {e}. Exiting.")
        sys.exit(1)

    print("\n*** Compass Calibration ***")
    print("Place the robot on a flat surface, away from large metal objects or magnets.")
    print("Slowly rotate the robot 360 degrees (at least one full turn) in place.")
    print("Try to keep it level while rotating.")
    print("Press Ctrl+C when you have completed the rotation(s).")
    skriv_logg("Instructions displayed. Waiting for user to start rotation and press Ctrl+C.")

    x_min = 32767
    x_max = -32768
    y_min = 32767
    y_max = -32768

    try:
        skriv_logg("Starting raw data collection loop.")
        while True:
            x, y = compass_module.read_raw_xy() # Use the new function from compass_module

            # Update min and max values
            x_min = min(x_min, x)
            x_max = max(x_max, x)
            y_min = min(y_min, y)
            y_max = max(y_max, y)

            # Print current, min, and max values for user feedback
            print(f"Raw X: {x}, Raw Y: {y} | X Range: [{x_min} to {x_max}], Y Range: [{y_min} to {y_max}]")

            time.sleep(0.05) # Small delay between readings

    except KeyboardInterrupt:
        print("\nCalibration process stopped by user.")
        skriv_logg("Raw data collection stopped by user (Ctrl+C).")

        # Calculate offsets
        offset_x = (x_min + x_max) / 2.0
        offset_y = (y_min + y_max) / 2.0

        print(f"\nCalculated Compass Offsets:")
        print(f"� Offset X: {offset_x:.2f}")
        print(f"� Offset Y: {offset_y:.2f}")
        skriv_logg(f"Calculated offsets: x={offset_x:.2f}, y={offset_y:.2f}")

        # Save offsets to file
        try:
            with open("kompas_offset.txt", "w") as f:
                f.write(f"{offset_x}\n")
                f.write(f"{offset_y}\n")
            print("\nOffset values saved to 'kompas_offset.txt'.")
            skriv_logg("Offsets saved to kompas_offset.txt.")
        except Exception as e:
            print(f"\nError saving offset file: {e}")
            skriv_logg(f"Error saving offsets: {e}")

    print("Calibration tool finished.")
    skriv_logg("Calibration tool finished.")

if __name__ == "__main__":
    main()