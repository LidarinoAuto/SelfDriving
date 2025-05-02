# forbedret_hindringslogikk.py

from sensorer import lidar
from sensorer import ultrasound
import time
import math
import logg
import visning.visualisering as vis

ROBOT_DIAMETER = 23.5  # cm
SIKKERHETSMARGIN = 2.0  # cm
SIKKERHETS_RADIUS = (ROBOT_DIAMETER / 2) + SIKKERHETSMARGIN  # cm

STEP = 100
ROTATE_STEP = 50.0
#SAFE_DISTANCE_LIDAR = SIKKERHETS_RADIUS
SAFE_DISTANCE_LIDAR = 10
SAFE_DISTANCE_ULTRASOUND = 8  # cm
DRIVE_TIME = 2.0  # sekunder
HEADING_TOLERANCE = 5.0  # grader
AAPNING_BREDDE = 15  # grader minimum �pning
MAKS_HULL = 3  # antall grader med "falsk" blokkering som tillates

state = "IDLE"
drive_start_time = 0
current_target_angle = 0

def ultralyd_blokkert():
    blokkert = False
    for sensor, distance in ultrasound.sensor_distances.items():
        if distance > 0 and distance < SAFE_DISTANCE_ULTRASOUND:
            logg.skriv_logg(f"[ULTRALYD] Sensor {sensor} blokkert, avstand {distance:.1f} cm.")
            blokkert = True
    return blokkert

def normalize_angle(angle):
    return int((angle + 180) % 360 - 180)

def finn_aapninger():
    fri = [False] * 360
    for angle, distance in lidar.scan_data:
        if isinstance(angle, (int, float)):
            angle = int(round(angle)) % 360
            if distance > SAFE_DISTANCE_LIDAR:
                fri[angle] = True

    # Glatt ut sm� hull
    for i in range(360):
        if not fri[i]:
            hull_start = i
            lengde = 0
            while lengde < MAKS_HULL and not fri[(i + lengde) % 360]:
                lengde += 1
            if lengde <= MAKS_HULL:
                for j in range(lengde):
                    fri[(i + j) % 360] = True

    apninger = []
    start = None
    for i in range(360 + AAPNING_BREDDE):
        idx = i % 360
        if fri[idx]:
            if start is None:
                start = idx
        else:
            if start is not None:
                slutt = (idx - 1) % 360
                bredde = (slutt - start) % 360 + 1
                if bredde >= AAPNING_BREDDE:
                    apninger.append((start, slutt))
                start = None

    return apninger

def finn_storste_aapning(heading):
    apninger = finn_aapninger()
    if not apninger:
        vis.sett_aapninger([], None)
        return None, None

    beste = max(apninger, key=lambda a: (a[1] - a[0]) % 360)
    bredde = (beste[1] - beste[0]) % 360
    midt = (beste[0] + bredde / 2) % 360
    vis.sett_aapninger(apninger, midt)
    logg.skriv_logg(f"[SOK] Valgt �pning {beste[0]}?{beste[1]}, midt: {midt:.1f}�")
    return midt, 100  # midt og dummyavstand

def heading_correction(current_heading, target_heading, k=0.5, max_omega=90):
    avvik = normalize_angle(target_heading - current_heading)
    omega = -k * avvik
    return max(-max_omega, min(max_omega, omega))

def autonom_logikk(heading):
    global state, drive_start_time, current_target_angle

    if state == "IDLE":
        if not lidar.is_path_clear() or ultralyd_blokkert():
            state = "SEARCH"
            logg.skriv_logg("[HINDRING] Starter s�k etter �pning.")
            return (0, 0, ROTATE_STEP)
        else:
            return (STEP, 0, 0.0)

    elif state == "SEARCH":
        vinkel, _ = finn_storste_aapning(heading)

        if vinkel is not None:
            delta = normalize_angle(vinkel - heading)
            current_target_angle = heading + delta
            drive_start_time = time.time()
            state = "DRIVING"
            logg.skriv_logg("[HINDRING] �pning funnet, starter kj�ring mot �pning.")
            omega = heading_correction(heading, current_target_angle)
            return (STEP, 0, omega)
        else:
            logg.skriv_logg("[HINDRING] Ingen �pning funnet, roterer videre.")
            current_target_angle = (heading + 30) % 360
            return (0, 0, 30.0)

    elif state == "DRIVING":
        if (time.time() - drive_start_time) < DRIVE_TIME:
            delta = normalize_angle(current_target_angle - heading)
            if abs(delta) > HEADING_TOLERANCE:
                omega = heading_correction(heading, current_target_angle)
                logg.skriv_logg(f"[KORRIGERING] Heading delta={delta:.1f}, omega={omega:.1f}")
                return (0, 0, omega)
            else:
                logg.skriv_logg("[DRIVING] Heading ok, kj�rer frem.")
                return (STEP, 0, 0.0)
        else:
            state = "IDLE"
            logg.skriv_logg(f"[TILBAKE TIL IDLE] Fremdrift ferdig. N�v�rende heading: {heading:.1f}�")
            return (STEP, 0, 0.0)
