import math
import time
from sensorer import kompas
from sensorer import mpu6050
import logg

# Filterparametre
GYRO_VEKT_HOY = 0.98      # Brukes ved rask rotasjon
GYRO_VEKT_LAV = 0.90      # Brukes n�r roboten er i ro/lav hastighet
ROTASJON_TERSKEL = 30     # grader/s for � veksle mellom lav/h�y gyro-vekt
KOMPASS_CORRECT_INTERVAL = 0.20  # sekunder mellom kompasskorreksjon
KOMPASS_JUSTERING = 270          # Juster hvis fysisk nord ikke stemmer

class HeadingTracker:
    def __init__(self):
        self.last_time = time.time()
        self.last_compass_update = 0
        self.fused_heading = 0
        self.initialized = False
        self.latest_compass = 0
        self.latest_gyro = 0

    def setup(self):
        kompas.setup_compass()
        mpu6050.setup_mpu6050()
        heading = kompas.read_heading()
        if heading != -1:
            self.fused_heading = (heading + KOMPASS_JUSTERING) % 360
            self.latest_compass = self.fused_heading
        else:
            self.fused_heading = 0
            self.latest_compass = 0
        self.initialized = True
        logg.skriv_logg(f"[HEADING] Init: heading={self.fused_heading:.1f}�")

    def update(self):
        if not self.initialized:
            self.setup()
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0 or dt > 1:  # beskytt mot store hopp
            dt = 0.02
        self.last_time = current_time

        # --- Integrer gyro (Z-akse) ---
        gyro_z = mpu6050.read_gyro_z()  # grader/sek
        self.latest_gyro = gyro_z
        delta_heading = gyro_z * dt
        self.fused_heading = (self.fused_heading + delta_heading) % 360

        # --- Kompasskorreksjon ---
        if (current_time - self.last_compass_update) > KOMPASS_CORRECT_INTERVAL:
            compass_heading = kompas.read_heading()
            if compass_heading != -1:
                compass_heading = (compass_heading + KOMPASS_JUSTERING) % 360
                self.latest_compass = compass_heading
                # Dynamisk vekt: mer gyro under h�y rotasjon
                gyro_rate = abs(gyro_z)
                if gyro_rate > ROTASJON_TERSKEL:
                    gyro_vekt = GYRO_VEKT_HOY
                else:
                    gyro_vekt = GYRO_VEKT_LAV
                kompass_vekt = 1 - gyro_vekt

                # Minimer wrap-around
                delta = ((compass_heading - self.fused_heading + 540) % 360) - 180
                correction = delta * kompass_vekt
                self.fused_heading = (self.fused_heading + correction) % 360

                logg.skriv_logg(
                    f"[HEADING] Kompass={compass_heading:.1f}� GyroZ={gyro_z:.2f}�/s ?={delta:.1f}� ? Korrigerer {correction:.2f}� ? {self.fused_heading:.1f}� (gyro_vekt={gyro_vekt:.2f})"
                )
            else:
                logg.skriv_logg("[HEADING] Kompassfeil, ingen korreksjon!")
            self.last_compass_update = current_time

        return self.fused_heading

    def get_heading(self):
        return self.fused_heading

    def get_compass(self):
        """Returnerer siste leste kompass-heading (for visning/debug)."""
        return self.latest_compass

    def get_gyro(self):
        """Returnerer siste gyro-z-verdi (for visning/debug)."""
        return self.latest_gyro

    def set_heading(self, heading):
        """For evt. nullstilling/reset av heading (f.eks. sett N = n�v�rende)."""
        self.fused_heading = heading % 360
        logg.skriv_logg(f"[HEADING] Nullstiller heading til {self.fused_heading:.1f}�")
