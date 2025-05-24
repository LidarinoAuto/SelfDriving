# heading.py (robust komplementærfilter og logging)
import math
import time
from sensorer import kompas
from sensorer import mpu6050
import logg

# Tuningparametre
GYRO_VEKT = 0.96        # Jo nærmere 1, jo mer stoles på gyro
KOMPASS_VEKT = 1 - GYRO_VEKT
KOMPASS_CORRECT_INTERVAL = 0.25   # sekunder mellom kompasskorreksjon (reduserer støy)
KOMPASS_JUSTERING = 270           # Tilpass etter fysisk plassering (juster om nord peker "feil vei")

class HeadingTracker:
    def __init__(self):
        self.last_time = time.time()
        self.last_compass_update = 0
        self.fused_heading = 0
        self.initialized = False

    def setup(self):
        kompas.setup_compass()
        mpu6050.setup_mpu6050()
        heading = kompas.read_heading()
        if heading != -1:
            # Juster for fysisk montering, bruk kompass som startpunkt
            self.fused_heading = (heading + KOMPASS_JUSTERING) % 360
        else:
            self.fused_heading = 0
        self.initialized = True
        logg.skriv_logg(f"[HEADING] Initiering: heading={self.fused_heading:.1f}°")

    def update(self):
        if not self.initialized:
            self.setup()
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # --- Integrer gyro (Z-akse) ---
        gyro_z = mpu6050.read_gyro_z()  # grader/sek
        delta_heading = gyro_z * dt
        self.fused_heading = (self.fused_heading + delta_heading) % 360

        # --- Kompasskorreksjon (kun hvis tiden er inne) ---
        if (current_time - self.last_compass_update) > KOMPASS_CORRECT_INTERVAL:
            compass_heading = kompas.read_heading()
            if compass_heading != -1:
                compass_heading = (compass_heading + KOMPASS_JUSTERING) % 360
                # Minimer "wrap-around" problem mellom 0/360
                delta = ((compass_heading - self.fused_heading + 540) % 360) - 180
                correction = delta * KOMPASS_VEKT
                self.fused_heading = (self.fused_heading + correction) % 360
                logg.skriv_logg(
                    f"[HEADING] Kompass={compass_heading:.1f}° GyroZ={gyro_z:.2f}°/s Δ={delta:.1f}° → Korrigerer med {correction:.2f}° til {self.fused_heading:.1f}°"
                )
            else:
                logg.skriv_logg("[HEADING] Kompassfeil, ingen korreksjon!")
            self.last_compass_update = current_time

        return self.fused_heading

    def get_heading(self):
        return self.fused_heading

    def set_heading(self, heading):
        """For evt. nullstilling/reset av heading (f.eks. sett N = nåværende)."""
        self.fused_heading = heading % 360
        logg.skriv_logg(f"[HEADING] Nullstiller heading til {self.fused_heading:.1f}°")

