# forbedret_hindringslogikk.py

from sensorer import lidar
from sensorer import ultrasound
import time
import math
import logg

# Robotens fysiske diameter i centimeter
ROBOT_DIAMETER = 23.5  # cm

# Ekstra sikkerhetsmargin
SIKKERHETSMARGIN = 2.0  # cm

# Beregnet sikker radius
SIKKERHETS_RADIUS = (ROBOT_DIAMETER / 2) + SIKKERHETSMARGIN  # cm

STEP = 100
ROTATE_STEP = 30.0
SAFE_DISTANCE_LIDAR = SIKKERHETS_RADIUS  # <-- Bruk sikker radius her!
SAFE_DISTANCE_ULTRASOUND = 8  # cm
DRIVE_TIME = 2.0  # sekunder vi forplikter oss til fremdrift

state = "IDLE"
drive_start_time = 0
current_target_angle = 0

#log = logging.getLogger()

def vurder_aapning():
    beste_vinkel = None
    beste_avstand = 0

    for angle, distance in lidar.scan_data:
        if distance > 0:
            a = (angle + 360) % 360
            if a <= 60 or a >= 300:
                if distance > beste_avstand:
                    beste_vinkel = angle
                    beste_avstand = distance

    if beste_avstand > (ROBOT_DIAMETER + SIKKERHETSMARGIN):
        logg.skriv_logg(f"[SOK] Beste aapning pa {beste_vinkel:.1f} grader, avstand {beste_avstand:.1f} cm.")
        return beste_vinkel, beste_avstand
    else:
        return None, None

def ultralyd_blokkert():
    blokkert = False
    for sensor, distance in ultrasound.sensor_distances.items():
        if distance > 0 and distance < SAFE_DISTANCE_ULTRASOUND:
            logg.skriv_logg(f"[ULTRALYD] Sensor {sensor} blokkert, avstand {distance:.1f} cm.")
            blokkert = True
    return blokkert


def autonom_logikk(heading):
    global state, drive_start_time, current_target_angle

    if state == "IDLE":
        if not lidar.is_path_clear() or ultralyd_blokkert():
            state = "SEARCH"
            logg.skriv_logg("[HINDRING] Starter sok etter aapning.")
            return (0, 0, ROTATE_STEP)
        else:
            return (STEP, 0, 0.0)

    elif state == "SEARCH":
        vinkel, avstand = vurder_aapning()
        if vinkel is not None:
            delta = ((vinkel - heading + 540) % 360) - 180
            current_target_angle = heading + delta
            drive_start_time = time.time()
            state = "DRIVING"
            logg.skriv_logg("[HINDRING] Aapning funnet, starter kjore mot aapning.")
            
            # La roboten snu korrekt med en gang
            MAX_ROTATE_SPEED = 90.0
            if delta > MAX_ROTATE_SPEED:
                omega = MAX_ROTATE_SPEED
            elif delta < -MAX_ROTATE_SPEED:
                omega = -MAX_ROTATE_SPEED
            else:
                omega = delta
    
            return (STEP, 0, omega)
        
        else:
            logg.skriv_logg("[HINDRING] Ingen aapning funnet, roterer sakte videre.")
            # Hvis ingen �pning funnet, roter sakte
            return (0, 0, 15.0)


    elif state == "DRIVING":
        if (time.time() - drive_start_time) < DRIVE_TIME:
            vinkel, avstand = vurder_aapning()
            if vinkel is not None:
                delta = ((vinkel - heading + 540) % 360) - 180
                if abs(delta) > 10:
            
                    # Juster hastighet mot m�lvinkel proporsjonalt
                    MAX_ROTATE_SPEED = 90.0  # eller en annen verdi du �nsker
                    if delta > MAX_ROTATE_SPEED:
                        omega = MAX_ROTATE_SPEED
                    elif delta < -MAX_ROTATE_SPEED:
                        omega = -MAX_ROTATE_SPEED
                    else:
                        omega = delta
                    logg.skriv_logg(f"[KORRIGERING] Avviker fra mål med {delta:.1} grader, korrigerer med omega={omega:.1f}.")
                    return (STEP, 0, omega)

            logg.skriv_logg("[DRIVING] Kjorer rett frem uten korreksjon.")
            return (STEP, 0, 0.0)
        else:
            state = "IDLE"
            logg.skriv_logg(f"[TILBAKE TIL IDLE] Kjoring mot aapning ferdig. Heading nå: {heading:.1f} grader.")
            return (STEP, 0, 0.0)

    else:
        state = "IDLE"
        return (0, 0, 0.0)
