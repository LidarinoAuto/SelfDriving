# heading_tracker.py
import math
import time
# IMPORT DE RIKTIGE MODULENE FRA DITT N V RENDE PROSJEKT
# Sannsynligvis disse:
from compass_module import read_compass, init_compass as init_compass_module # Antar du kaller init i modulen
from imu_module import read_imu_data, init_imu as init_imu_module # Antar du kaller init i modulen


# Konstanter for Complementary Filter
# Juster disse om n dvendig basert p  testing
GYRO_WEIGHT = 0.90  # H yere verdi betyr mer tillit til gyro (jevnere, men drifter)
COMPASS_WEIGHT = 1.0 - GYRO_WEIGHT # Lavere verdi betyr mindre tillit til kompass (mindre st y fra kompass, men tregere korreksjon)

# EKSEMPEL: Hvis kompasset ditt trenger en fast justering for   peke Nord der du vil
KOMPASS_JUSTERING = 270 # Dette var i den andre koden. Juster/fjern basert p  dine behov.

class HeadingTracker:
    def __init__(self):
        self.last_time = time.time()
        self.fused_heading = 0.0 # Start med 0 som standard

    def setup(self):
        # Initialiser sensorer (hvis ikke allerede gjort i main.py)
        # Pass p  at init_compass_module og init_imu_module finnes/kalles riktig
        # i ditt n v rende prosjekt
        # init_compass_module() # Kanskje dette gj res i main.py allerede?
        # init_imu_module()     # Kanskje dette gj res i main.py allerede?

        # Sett innledende f sed heading basert p  kompass
        # Det er viktig at kompasset gir en stabil m ling N R ROBOTEN ST R STILLE her.
        # V r kalibrering skal hjelpe med dette.
        heading = read_compass() # Bruk read_compass fra din compass_module

        if heading != -1.0: # Sjekk at kompasslesing var vellykket
            self.fused_heading = (heading + KOMPASS_JUSTERING) % 360 # Bruk KOMPASS_JUSTERING om n dvendig
            #self.fused_heading = heading # Bruk raw compass heading hvis ingen justering trengs
        else:
            self.fused_heading = 0.0 # Fallback til 0 hvis kompasset feiler ved start
            # Logg en feilmelding her om initial kompasslesing feilet?

        self.last_time = time.time() # Start tidtaking ETTER initialisering

    def update(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Forhindre store dt ved debugging / pause
        if dt > 0.1: # Sett en maksgrense for dt
            dt = 0.1
            # Kanskje resette gyro-integrasjonen helt her?
            # Eller logge en advarsel

        # --- GYRO INTEGRASJON ---
        gyro_z = read_imu_data()
        self.fused_heading = (self.fused_heading + gyro_z * dt) % 360
        # H ndter negativ heading fra modulo
        if self.fused_heading < 0:
            self.fused_heading += 360

        # --- KOMPASS KORREKSJON ---
        # Bruk read_compass fra din compass_module. S rg for den returnerer 0-360 grader ETTER offset/kompensasjon.
        compass_heading = read_compass()

        if compass_heading != -1.0: # Sjekk at kompasslesing var vellykket
            compass_heading = (compass_heading + KOMPASS_JUSTERING) % 360 # Bruk justering om n dvendig

            # Beregn korteste vinkel mellom f sed og kompass heading (-180 til +180)
            delta = ((compass_heading - self.fused_heading + 540.0) % 360.0) - 180.0

            # Bruk en liten del av delta til   korrigere f sed heading (komplement rfilter)
            self.fused_heading = (self.fused_heading + delta * COMPASS_WEIGHT) % 360

            # H ndter negativ heading fra modulo
            if self.fused_heading < 0:
                self.fused_heading += 360

        # Returner den oppdaterte f sede headingen
        return self.fused_heading

    def get_heading(self):
        # Returner gjeldende f sed heading
        return self.fused_heading