# Filename: Heading_Tracker.py
import time
import math
from logging_utils import skriv_logg
# FJERN ALLE IMPORTER SOM KOMMER FRA IMU_MODULE ELLER COMPASS_MODULE HER
# IKKE IMPORTER read_imu_data, init_imu, read_compass_data, init_compass ETC.
# Disse mottas n i __init__ metoden.


class HeadingTracker:
    """
    Tracks the robot's fused heading by combining Gyro (short-term)
    and Compass (long-term) data using a complementary filter.
    Receives sensor reading functions during initialization.
    """
    # Vektfaktor for Gyro. Hold denne P  0.90
    GYRO_WEIGHT = 0.9
    COMPASS_WEIGHT = 1.0 - GYRO_WEIGHT # Beregnes automatisk

    # Statisk korreksjon for kompasset (grader) basert p  manuell kalibrering
    # Dette offsetet justerer den r  kompass-headingen til   peke Nord (0 grader)
    # n r roboten fysisk peker Nord.
    STATIC_COMPASS_OFFSET = -105.0 # <-------- DEFINER STATISK KORREKSJON HER


    def __init__(self, imu_data_reader, compass_data_reader): # <--- MOTTA LESEFUNKSJONER HER
        """
        Initializes the HeadingTracker with sensor reading functions.
        Performs initial compass read for first fused heading.
        :param imu_data_reader: Function that returns Z-axis angular velocity (deg/sec), or 0.0 if IMU not ready/error.
        :param compass_data_reader: Function that returns Compass heading (0-359.9 degrees), or -1.0 on error.
        """
        # Lagre referansene til funksjonene som HeadingTracker skal bruke
        self.imu_data_reader = imu_data_reader
        self.compass_data_reader = compass_data_reader

        self._fused_heading = 0.0  # Fused heading, initialized (this will be the UNCUTTIGED fused heading)
        self._last_time = time.time() # Time of the last update

        # --- Initial calibration/synchronization ---
        # Les kompass ved start for en grov initial heading.
        # Dette skjer KUN n r objektet opprettes.
        initial_compass_heading_raw = self.compass_data_reader() # Bruk den sendte funksjonen

        if initial_compass_heading_raw != -1.0: # Forutsatt -1.0 indikerer feil i compass_module
            # Initial fused heading set directly from the raw compass reading
            self._fused_heading = initial_compass_heading_raw # Store the raw fused heading
            # Log the raw initial heading
            skriv_logg(f"HeadingTracker initialized. Initial fused heading set to raw compass value: {self._fused_heading:.1f} ")
        else:
            skriv_logg("Warning: HeadingTracker initialized, but could not get initial compass heading. Fused heading set to 0.")
            self._fused_heading = 0.0 # Fallback til 0 hvis kompass feiler

        # Oppdater last_time etter initialisering
        self._last_time = time.time()


    def update(self):
        """
        Updates the fused heading using new sensor readings from the
        functions provided during initialization. Should be called
        frequently (e.g., in the main loop or before PID calculation).
        """
        current_time = time.time()
        dt = current_time - self._last_time # Time elapsed since last update

        # Handle very small or zero dt to avoid division by zero or large gyro updates
        if dt < 0.0001: # Minimum dt, just skip update if too fast
            return # Skip update if time difference is negligible

        self._last_time = current_time # Update last update time for the next iteration

        # --- Get new sensor data using the passed functions ---
        # Kall funksjonene som ble sendt inn til __init__
        gyro_rate = self.imu_data_reader()
        compass_heading_raw = self.compass_data_reader() # This is the heading from compass_module (using atan2(-Y,X))


        # --- Complementary Filter Logic ---
        # Integrer gyro for a estimere endring i heading over dt
        # Anta at gyro_rate er i grader/sekund
        gyro_delta_heading = gyro_rate * dt

        # Oppdater fasede heading ved a legge til gyro-estimatet
        # Handter wrap-around n r headingen gar forbi 360/0
        fused_heading_gyro_updated = (self._fused_heading + gyro_delta_heading) % 360.0
        # Juster til a v re i [0, 360)
        if fused_heading_gyro_updated < 0:
            fused_heading_gyro_updated += 360


        # Bruk kompasset for a korrigere drift (langsiktig referanse)
        # Gj r dette kun hvis kompass data er tilgjengelig
        if compass_heading_raw != -1.0: # Forutsatt -1.0 indikerer feil i compass_module
            # Beregn feilen mellom gjeldende fused (gyro-oppdatert) og kompass-headingen
            # Bruk smidig feilberegning for a handtere wrap-around (korteste vei)
            # error = ((target - current + 540.0) % 360.0) - 180.0
            # Her er target = compass_heading_raw, current = fused_heading_gyro_updated
            heading_difference = ((compass_heading_raw - fused_heading_gyro_updated + 540.0) % 360.0) - 180.0

            # Korriger fused heading basert p heading_difference og kompassets vekt
            # Denne korreksjonen skjer kun hvis kompass-data er tilgjengelig
            self._fused_heading = fused_heading_gyro_updated + (heading_difference * self.COMPASS_WEIGHT)
        else:
            # Hvis kompass data ikke er tilgjengelig, stoler vi kun p gyro for denne oppdateringen
            self._fused_heading = fused_heading_gyro_updated
            # skriv_logg("Warning: Compass data not available. Fused heading updated using only gyro.") # Unng spamming

    def get_heading(self):
        """
        Returns the current fused heading in degrees (0-359.9),
        with the static compass offset applied.
        """
        # Apply the static offset to the fused heading before returning
        corrected_heading = (self._fused_heading + self.STATIC_COMPASS_OFFSET + 360.0) % 360.0 # <-------- LEGG TIL KORREKSJONEN HER
        return corrected_heading # Returner den korrigerte headingen
