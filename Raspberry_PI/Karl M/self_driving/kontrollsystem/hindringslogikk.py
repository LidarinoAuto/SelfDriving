# hindringslogikk.py
from sensorer import lidar
from sensorer import ultrasound
import time

STEP = 100
ROTATE_STEP = 30.0
SAFE_DISTANCE_LIDAR = 19  # cm
SAFE_DISTANCE_ULTRASOUND = 8  # cm
SEARCH_TIMEOUT = 10  # sekunder

search_mode = False
search_start_time = 0

# --- Logging ---
LOG_FILE = "hindringslogg.txt"

def logg(message):
    timestamp = time.strftime("%H:%M:%S")
    with open(LOG_FILE, "a") as f:
        f.write(f"[{timestamp}] {message}\n")

def is_path_clear():
    """Sjekker om det er trygt å kjøre fremover"""
    path_clear = lidar.is_path_clear()

    front_blocked = False
    if ultrasound.sensor_distances["front_left"] > 0 and ultrasound.sensor_distances["front_left"] < SAFE_DISTANCE_ULTRASOUND:
        front_blocked = True
    if ultrasound.sensor_distances["front_right"] > 0 and ultrasound.sensor_distances["front_right"] < SAFE_DISTANCE_ULTRASOUND:
        front_blocked = True

    logg(f"Path clear (LIDAR): {path_clear}, Front blocked (Ultrasound): {front_blocked}")
    return path_clear and not front_blocked

def autonom_logikk():
    global search_mode, search_start_time

    if not search_mode:
        # Normal fremoverkjøring
        if is_path_clear():
            logg("Kjører fremover (vei fri)")
            return (STEP, 0, 0.0)
        else:
            # Starter søk etter åpen vei
            search_mode = True
            search_start_time = time.time()
            logg("Hindring foran - starter søkemodus")
            return (0, 0, ROTATE_STEP)
    else:
        # Søkemodus aktiv
        if is_path_clear():
            # Fant åpen vei, kjør fremover
            search_mode = False
            logg("Fant åpning - kjører fremover")
            return (STEP, 0, 0.0)
        elif (time.time() - search_start_time) > SEARCH_TIMEOUT:
            # Timeout uten å finne vei
            search_mode = False
            logg("Timeout - ingen åpning funnet, stopper robot")
            return (0, 0, 0.0)
        else:
            logg("Fortsetter søk - roterer")
            return (0, 0, ROTATE_STEP)
