# Filename: ultrasound_module.py
# Module for reading distance from HC-SR04 ultrasound sensors

import RPi.GPIO as GPIO
import time
import sys # Import sys for error handling
from logging_utils import skriv_logg

# --- GPIO Pin Configuration (BCM numbering) ---
# Define the pins for each sensor (Trigger and Echo)
# These are just example pins. ADJUST THESE TO MATCH YOUR WIRING IF DIFFERENT.
# Based on your provided code, these pins are:
# Front Left (FV): Trigger=9, Echo=8
# Front Right (FH): Trigger=7, Echo=6
# Back Left (BV): Trigger=23, Echo=24
# Back Right (BH): Trigger=10, Echo=11

US_PINS = {
    0: (9, 8),   # Front Left (FV) - Based on your code's first front sensor
    1: (7, 6),   # Front Right (FH) - Based on your code's second front sensor
    2: (23, 24), # Back Left (BV) - Based on your code's first back sensor
    3: (10, 11)  # Back Right (BH) - Based on your code's second back sensor
}

# --- Setup GPIO ---
def setup_ultrasound_gpio():
    """Sets up GPIO pins for all ultrasound sensors."""
    skriv_logg("Setting up Ultrasound GPIO pins...")
    try:
        GPIO.setmode(GPIO.BCM) # Use Broadcom pin-numbering scheme
        GPIO.setwarnings(False) # Disable GPIO warnings

        # Collect all trigger and echo pins from the dictionary
        all_trig_pins = [pins[0] for pins in US_PINS.values()]
        all_echo_pins = [pins[1] for pins in US_PINS.values()]

        # Setup all trigger pins as output
        for trig_pin in all_trig_pins:
            GPIO.setup(trig_pin, GPIO.OUT)
            GPIO.output(trig_pin, False) # Ensure trigger is low initially

        # Setup all echo pins as input
        for echo_pin in all_echo_pins:
             GPIO.setup(echo_pin, GPIO.IN)

        # Give sensors a moment to settle
        time.sleep(0.5)
        skriv_logg("Ultrasound GPIO setup complete.")
        return True # Indicate success

    except Exception as e:
        skriv_logg(f"Error during Ultrasound GPIO setup: {e}")
        # Decide if this is a critical error forcing exit or just a warning
        # For now, skriv_logg error and allow program to continue if possible,
        # but readings might be unreliable.
        # It might be better to raise the exception or return False and handle in main.
        # For consistency with initialize_systems in main, let's skriv_logg and return False on error
        return False


# --- Read Distance from a Single Sensor ---
def read_single_ultrasound(sensor_id, timeout=0.03):
    """
    Reads distance from a single ultrasound sensor by its ID.
    Returns the distance in centimeters or float('inf') on error or timeout.
    """
    if sensor_id not in US_PINS:
        # skriv_logg(f"Error: Invalid sensor ID {sensor_id}") # Optional debug skriv_logg
        return float('inf') # Return infinity for invalid ID

    trigger_pin, echo_pin = US_PINS[sensor_id]

    try:
        # Ensure trigger is low before pulsing
        GPIO.output(trigger_pin, False)
        time.sleep(0.000002) # Wait for 2 microseconds

        # Send 10us pulse to trigger
        GPIO.output(trigger_pin, True)
        time.sleep(0.00001) # 10 microseconds pulse
        GPIO.output(trigger_pin, False)

        pulse_start = time.time()
        pulse_end = time.time()
        timeout_start = time.time()

        # Wait for echo start (pulse goes high)
        while GPIO.input(echo_pin) == 0:
            pulse_start = time.time()
            if time.time() - timeout_start > timeout:
                # skriv_logg(f"Timeout waiting for echo start on sensor {sensor_id}") # Optional debug
                return float('inf') # Return infinity on timeout

        # Wait for echo end (pulse goes low)
        timeout_start = time.time() # Reset timeout for the falling edge wait
        while GPIO.input(echo_pin) == 1:
            pulse_end = time.time()
            if time.time() - timeout_start > timeout:
                 # skriv_logg(f"Timeout waiting for echo end on sensor {sensor_id}") # Optional debug
                 return float('inf') # Return infinity on timeout

        # Calculate pulse duration in seconds
        pulse_duration = pulse_end - pulse_start

        # Speed of sound is ~343 meters/second or 34300 cm/s.
        # Distance = (Time x Speed of Sound) / 2
        distance_cm = (pulse_duration * 34300) / 2

        # Basic range check based on sensor capability (typically 2cm to 400cm)
        if distance_cm <= 2.0 or distance_cm > 400.0:
             # skriv_logg(f"Invalid range reading {distance_cm:.2f} cm from sensor {sensor_id}") # Optional debug
             return float('inf') # Consider out-of-range as infinity


        return distance_cm

    except Exception as e:
        # skriv_logg(f"Error reading sensor {sensor_id}: {e}") # Optional debug
        return float('inf') # Return infinity on error


# --- Check All Sensors and Get Triggered Info ---
def get_triggered_ultrasound_info(threshold_cm):
    """
    Reads all ultrasound sensors sequentially and identifies which ones are below a threshold.
    Based on the structure needed by the new main.py.
    Returns a tuple: (list of triggered sensor IDs, dictionary of distances).
    Distances are in CENTIMETERS.
    """
    triggered_sensors = []
    distances = {}

    # Read each sensor by its ID
    for sensor_id in US_PINS.keys():
        distance = read_single_ultrasound(sensor_id)
        distances[sensor_id] = distance # Store distance even if not triggered

        # Check if distance is below the threshold (and is a valid reading, not infinity)
        if distance != float('inf') and distance < threshold_cm:
            triggered_sensors.append(sensor_id)
            # Optional: skriv_logg which sensor triggered below threshold
            # skriv_logg(f"Ultrasound sensor {sensor_id} triggered! Distance: {distance:.2f} cm (Threshold: {threshold_cm:.2f} cm)")

        # Small delay between sensor readings to avoid interference
        time.sleep(0.06) # Delay based on sensor cycle time (min ~50ms)

    return triggered_sensors, distances


# --- Cleanup GPIO ---
def cleanup_ultrasound_gpio():
    """Cleans up GPIO pins used by ultrasound sensors."""
    skriv_logg("Cleaning up GPIO for ultrasound...")
    try:
        # Collect all pins to clean up
        all_pins = [pin for pins in US_PINS.values() for pin in pins]
        GPIO.cleanup(all_pins) # Clean up only the pins we used
        skriv_logg("Ultrasound GPIO cleanup complete.")
    except Exception as e:
        skriv_logg(f"Error during Ultrasound GPIO cleanup: {e}")


# This module is designed to be imported and used by main.py.
# It does not have its own if __name__ == "__main__": block to run independently.