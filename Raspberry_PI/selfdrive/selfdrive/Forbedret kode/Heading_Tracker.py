# Filename: Heading_Tracker.py
# Class for tracking robot's heading by fusing compass and gyro data

import time
import math
import threading
from logging_utils import skriv_logg # Assuming you have this utility

# --- IMPORT NECESSARY MODULES FOR READING SENSOR DATA ---\
# Import the actual sensor modules so their functions can be called directly.
import imu_module # Import the imu_module
import compass_module # Import the compass_module
# Assumes these modules contain the functions read_imu_data() and read_compass_data()
# with the expected return types:
# - imu_module.read_imu_data(): Should return corrected gyro Z value in degrees per second (float), or None on error/not initialized.
# - compass_module.read_compass_data(): Should return heading in degrees (0-360 float), or -1.0 on error.
# --- END IMPORT NECESSARY MODULES ---


class HeadingTracker:
    """
    Tracks the robot's heading by fusing compass data with gyro data.
    Compass provides absolute heading, gyro provides relative change.
    Uses a complementary filter for fusion.
    Reads sensor data directly from imu_module and compass_module.
    """
    def __init__(self, imu_data_reader, compass_data_reader, alpha=0.1, global_offset_degrees=0.0):
        """
        Initializes the HeadingTracker.

        Args:
            imu_data_reader: Function that returns corrected gyro Z in deg/s.
            compass_data_reader: Function that returns compass heading (0-360 float), or -1.0 on error.
            alpha: Complementary filter alpha value (0 to 1). Higher alpha favors gyro.
            global_offset_degrees: A constant offset to apply to all reported headings
                                   to align with a specific North reference (e.g., true North).
                                   Subtract this value from the raw heading.
        """
        self._imu_data_reader = imu_data_reader
        self._compass_data_reader = compass_data_reader
        self.alpha = alpha # Filter coefficient

        self._current_heading = -1.0 # Store the current fused heading (0-360). -1.0 means not initialized.
        self._last_time = time.time() # Time of the last update

        self.global_offset_degrees = global_offset_degrees # Store the global offset

        # Use a lock for thread-safe access to _current_heading if update/get_heading
        # are called from different threads (e.g., main loop and a data collection thread)
        self._lock = threading.Lock()

        skriv_logg("HeadingTracker: Initializing...")

        # Attempt to get initial heading from compass
        initial_compass_heading = -1.0
        attempts = 0
        max_attempts = 10 # Try a few times to get a valid reading
        while initial_compass_heading == -1.0 and attempts < max_attempts:
            initial_compass_heading = self._compass_data_reader()
            if initial_compass_heading == -1.0:
                # skriv_logg("HeadingTracker: Waiting for valid compass reading...")
                time.sleep(0.1) # Wait a bit for sensor
                attempts += 1

        with self._lock:
            if initial_compass_heading != -1.0:
                self._current_heading = initial_compass_heading # Initialize with compass reading
                # The initial heading is stored raw, offset is applied in get_heading()
                skriv_logg(f"HeadingTracker: Initialized heading with first valid compass reading: {self._current_heading:.1f} (raw)")
            else:
                skriv_logg("HeadingTracker: Failed to get initial compass reading after multiple attempts.")
                # _current_heading remains -1.0, indicating not initialized based on compass

        self._last_time = time.time() # Reset timer after initialization attempt
        skriv_logg(f"HeadingTracker: __init__ finished. Initial _current_heading is: {self._current_heading:.1f}")

    def update(self):
        """
        Updates the fused heading using new sensor data and complementary filter.
        Should be called regularly in the main loop or a dedicated sensor thread.
        """
        if self._current_heading == -1.0:
            # If not initialized yet, try to get an initial compass reading
            initial_compass_heading = self._compass_data_reader()
            if initial_compass_heading != -1.0:
                 with self._lock:
                    self._current_heading = initial_compass_heading # Initialize with compass reading
                    # Initial heading stored raw
                    skriv_logg(f"HeadingTracker: Initialized heading with valid compass reading during update: {self._current_heading:.1f} (raw)")
                    self._last_time = time.time() # Reset time
            else:
                 # skriv_logg("HeadingTracker: Still waiting for initial compass reading.")
                 return # Cannot update without initial heading


        current_time = time.time()
        dt = current_time - self._last_time
        self._last_time = current_time

        # Avoid division by zero or very small dt
        if dt == 0:
             return # Skip update if no time has passed


        # Read sensor data
        gyro_z_deg_s = self._imu_data_reader() # Get corrected gyro Z (already sign-fixed in imu_module)
        compass_heading = self._compass_data_reader() # Get compass heading (0-360 or -1.0)

        # --- Fusion Logic (Complementary Filter) ---
        with self._lock: # Use lock while accessing/modifying _current_heading
            # Ensure we have valid gyro data
            if gyro_z_deg_s is not None:
                 # Predict new heading based on gyro
                 gyro_delta_heading = gyro_z_deg_s * dt
                 predicted_heading = (self._current_heading + gyro_delta_heading) % 360.0
                 if predicted_heading < 0:
                     predicted_heading += 360

                 # If compass reading is valid, fuse with gyro prediction
                 if compass_heading != -1.0:
                     # Ensure compass heading is in 0-360 range before fusing
                     compass_heading = compass_heading % 360.0
                     if compass_heading < 0:
                         compass_heading += 360

                     # Calculate difference between predicted heading and compass heading
                     # Handle wrap-around for the difference
                     error = compass_heading - predicted_heading
                     if error > 180:
                         error -= 360
                     elif error < -180:
                         error += 360

                     # Apply correction from compass to the predicted heading
                     # The new heading is a blend of the gyro prediction and the compass correction
                     # (1-alpha) * compass + alpha * gyro_predicted is a common form, but
                     # applying the compass correction to the gyro prediction is also a form.
                     # Let's use the form: predicted + (1-alpha) * error
                     fused_heading = (predicted_heading + (1.0 - self.alpha) * error) % 360.0

                     if fused_heading < 0:
                         fused_heading += 360

                     self._current_heading = fused_heading # Update stored heading (raw)

                 else:
                     # If compass reading is invalid, rely solely on gyro (will drift over time)
                     # In this case, predicted_heading is the new heading
                     self._current_heading = predicted_heading # Update stored heading (raw)


            # If gyro data is also invalid (shouldn't happen if initialized), _current_heading remains the last valid fused heading
            # If _current_heading is still -1.0, we wait for a valid compass reading in the initialization check at start of update.


            # Optional: Log fused heading for debugging
            # skriv_logg(f"HeadingTracker: Fused Heading after update (raw): {self._current_heading:.2f} ")

    def get_heading(self):
        """
        Returns the current fused heading in degrees (0-359.9), adjusted by the global offset.
        Returns -1.0 if not initialized.
        """
        with self._lock: # Use lock for thread-safe access
            if self._current_heading == -1.0:
                return -1.0 # Return -1.0 if not initialized

            # Retrieve the stored heading (which is the raw fused heading)
            raw_heading = self._current_heading

            # Apply global offset and normalize to 0-360
            adjusted_heading = (raw_heading - self.global_offset_degrees + 360.0) % 360.0

            # --- DEBUG LOGGING INSIDE GET_HEADING ---
            skriv_logg(f"HeadingTracker: get_heading() called. Raw: {raw_heading:.1f}, Offset: {self.global_offset_degrees:.1f}, Returning: {adjusted_heading:.1f}")
            # --- END DEBUG LOGGING ---

            return adjusted_heading

    def reset_heading(self, new_heading=0.0):
        """Resets the current fused heading to a specific value (adjusted for offset)."""
        with self._lock: # Use lock for thread-safe access
            # When resetting to a "world-aligned" heading (like 0 for North),
            # we need to store it internally as raw_heading = world_aligned_heading + offset
            self._current_heading = (new_heading + self.global_offset_degrees + 360.0) % 360.0
            skriv_logg(f"HeadingTracker: Heading reset to {new_heading:.1f} (world). Stored raw: {self._current_heading:.1f}")