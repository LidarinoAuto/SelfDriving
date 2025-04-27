# hindringslogikk.py
from sensorer import lidar
from sensorer import ultrasound
from kontrollsystem.logg import skriv_logg
import time

STEP = 100  # Bevegelsessteg fremover
ROTATE_STEP = 30.0  # Rotasjonssteg
SAFE_DISTANCE_LIDAR = 50  # cm
SAFE_DISTANCE_ULTRASOUND = 20  # cm
SEARCH_TIMEOUT = 10  # sekunder

search_mode = False
search_start_time = 0

def is_path_clear():
    """Sjekker om det er trygt � kj�re fremover."""
    path_clear = lidar.is_path_clear()

    front_blocked = False
    if ultrasound.sensor_distances["front_left"] > 0 and ultrasound.sensor_distances["front_left"] < SAFE_DISTANCE_ULTRASOUND:
        front_blocked = True
    if ultrasound.sensor_distances["front_right"] > 0 and ultrasound.sensor_distances["front_right"] < SAFE_DISTANCE_ULTRASOUND:
        front_blocked = True

    return path_clear and not front_blocked

def autonom_logikk(fused_heading):
    """Autonom kj�relogikk basert p� sensorer."""
    global search_mode, search_start_time

    if not search_mode:
        if is_path_clear():
            return (STEP, 0, 0.0)
        else:
            # Starter s�k
            search_mode = True
            search_start_time = time.time()
            skriv_logg(f"[HINDRING]: Starter s�k etter �pning. N�v�rende heading: {fused_heading:.2f} grader")
            return (0, 0, ROTATE_STEP)
    else:
        if is_path_clear():
            skriv_logg(f"[HINDRING]: �pning funnet! Kj�rer fremover. Heading: {fused_heading:.2f} grader")
            search_mode = False
            return (STEP, 0, 0.0)
        elif (time.time() - search_start_time) > SEARCH_TIMEOUT:
            skriv_logg(f"[HINDRING]: S�ketid utl�pt. Stopper robot. Heading: {fused_heading:.2f} grader")
            search_mode = False
            return (0, 0, 0.0)
        else:
            # Fortsetter � rotere
            skriv_logg(f"[HINDRING]: S�ker fortsatt etter �pning... Heading: {fused_heading:.2f} grader")
            return (0, 0, ROTATE_STEP)
