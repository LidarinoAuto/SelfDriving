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

def is_path_clear():
    path_clear = lidar.is_path_clear()

    front_blocked = False
    if ultrasound.sensor_distances["front_left"] > 0 and ultrasound.sensor_distances["front_left"] < SAFE_DISTANCE_ULTRASOUND:
        front_blocked = True
    if ultrasound.sensor_distances["front_right"] > 0 and ultrasound.sensor_distances["front_right"] < SAFE_DISTANCE_ULTRASOUND:
        front_blocked = True

    return path_clear and not front_blocked

def autonom_logikk(current_heading):  # <--- Legg til current_heading her!
    global search_mode, search_start_time

    if not search_mode:
        if is_path_clear():
            logg("[HINDRING]: Klar bane. Kjører fremover.")
            return (STEP, 0, 0.0)
        else:
            search_mode = True
            search_start_time = time.time()
            logg(f"[HINDRING]: Starter søk etter åpning. Heading: {current_heading:.1f} grader")
            return (0, 0, ROTATE_STEP)
    else:
        if is_path_clear():
            logg(f"[HINDRING]: Åpning funnet! Heading: {current_heading:.1f} grader")
            search_mode = False
            return (STEP, 0, 0.0)
        elif (time.time() - search_start_time) > SEARCH_TIMEOUT:
            logg("[HINDRING]: Timeout. Stopper robot.")
            search_mode = False
            return (0, 0, 0.0)
        else:
            return (0, 0, ROTATE_STEP)

def logg(tekst):
    with open("hindringslogg.txt", "a") as f:
        tidspunkt = time.strftime("%Y-%m-%d %H:%M:%S")
        f.write(f"[{tidspunkt}] {tekst}\n")
