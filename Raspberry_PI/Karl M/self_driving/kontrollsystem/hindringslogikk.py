# hindringslogikk.py

from sensorer import lidar
from sensorer import ultrasound
import time
import math
import logging

# Bevegelse-konstanter
STEP = 100
ROTATE_SPEED = 30.0  # grader per sekund
SAFE_DISTANCE_LIDAR = 19  # cm
SAFE_DISTANCE_ULTRASOUND = 8  # cm
SEARCH_TIMEOUT = 10  # sekunder

# Intern tilstand
search_mode = False
search_start_time = 0
best_angle = None
rotation_direction = 1  # 1 = mot klokken, -1 = med klokken

# Sett opp logging
logging.basicConfig(filename="hindringslogikk.log", level=logging.INFO)

def is_front_clear():
    """Sjekk om fronten er fri basert på ultralydsensorene"""
    front_left_clear = ultrasound.sensor_distances.get("front_left", 999) > SAFE_DISTANCE_ULTRASOUND
    front_right_clear = ultrasound.sensor_distances.get("front_right", 999) > SAFE_DISTANCE_ULTRASOUND
    return front_left_clear and front_right_clear


def find_best_opening():
    """Finn vinkelen med mest fri plass"""
    max_distance = 0
    best_angle_local = 0
    for angle, distance in lidar.scan_data:
        if distance > max_distance and distance > SAFE_DISTANCE_LIDAR * 10:  # mm
            max_distance = distance
            best_angle_local = angle
    logging.info(f"[SOK] Beste aapning funnet pa {best_angle_local} grader med avstand {max_distance/10:.1f} cm.")
    return best_angle_local


def decide_rotation_direction(current_heading, target_angle):
    """Bestem rotasjonsretning basert på korteste vei"""
    delta = (target_angle - current_heading + 360) % 360
    if delta > 180:
        return -1  # Roter med klokken
    else:
        return 1   # Roter mot klokken


def autonom_logikk(current_heading):
    global search_mode, search_start_time, best_angle, rotation_direction

    if not search_mode:
        # Normal fremoverkjøring
        if lidar.is_path_clear() and is_front_clear():
            return (STEP, 0, 0.0)
        else:
            # Start søk
            search_mode = True
            search_start_time = time.time()
            best_angle = find_best_opening()
            rotation_direction = decide_rotation_direction(current_heading, best_angle)
            logging.info("[HINDRING] Starter sok etter aapning.")
            return (0, 0, rotation_direction * ROTATE_SPEED)

    else:
        # Vi er i søkemodus
        if (time.time() - search_start_time) > SEARCH_TIMEOUT:
            search_mode = False
            logging.warning("[HINDRING] Timeout i sok. Stopper robot.")
            return (0, 0, 0.0)

        # Beregn forskjell fra ønsket vinkel
        delta = (best_angle - current_heading + 360) % 360
        if delta > 180:
            delta -= 360

        if abs(delta) < 15:
            # Åpningen peker fremover
            if is_front_clear():
                logging.info("[HINDRING] Aapning foran funnet. Kjorer fremover.")
                search_mode = False
                return (STEP, 0, 0.0)

        # Fortsett å rotere
        return (0, 0, rotation_direction * ROTATE_SPEED)
