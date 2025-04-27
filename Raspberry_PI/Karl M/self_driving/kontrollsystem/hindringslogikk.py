# forbedret_hindringslogikk.py
from sensorer import lidar
from sensorer import ultrasound
import time
import math
import logging

STEP = 100
ROTATE_STEP = 30.0
SAFE_DISTANCE_LIDAR = 19  # cm
SAFE_DISTANCE_ULTRASOUND = 8  # cm
ROBOT_DIAMETER = 23.5  # cm
DRIVE_TIME = 2.0  # sekunder vi forplikter oss til fremdrift

state = "IDLE"
drive_start_time = 0
current_target_angle = 0

log = logging.getLogger()


def vurder_aapning():
    beste_vinkel = None
    beste_avstand = 0

    for angle, distance in lidar.scan_data:
        if distance > 0:
            # Se i et "vifte"-område foran (ca +-30 grader)
            a = (angle + 360) % 360
            if a <= 60 or a >= 300:
                if distance / 10.0 > beste_avstand:
                    beste_vinkel = angle
                    beste_avstand = distance / 10.0

    if beste_avstand > (ROBOT_DIAMETER + SAFE_DISTANCE_LIDAR):
        log.info(f"[SOK] Beste aapning pa {beste_vinkel:.1f} grader, avstand {beste_avstand:.1f} cm.")
        return beste_vinkel, beste_avstand
    else:
        return None, None


def ultralyd_blokkert():
    blokkert = False
    for sensor, distance in ultrasound.sensor_distances.items():
        if distance > 0 and distance < SAFE_DISTANCE_ULTRASOUND:
            blokkert = True
    return blokkert


def autonom_logikk(heading):
    global state, drive_start_time, current_target_angle

    if state == "IDLE":
        if not lidar.is_path_clear() or ultralyd_blokkert():
            state = "SEARCH"
            log.info("[HINDRING] Starter sok etter aapning.")
            return (0, 0, ROTATE_STEP)
        else:
            return (STEP, 0, 0.0)

    elif state == "SEARCH":
        vinkel, avstand = vurder_aapning()
        if vinkel is not None:
            # Bestem retning mot beste aapning
            delta = ((vinkel - heading + 540) % 360) - 180
            current_target_angle = heading + delta
            drive_start_time = time.time()
            state = "DRIVING"
            log.info("[HINDRING] Aapning funnet, starter kjore mot aapning.")
            return (STEP, 0, 0.0)
        else:
            return (0, 0, ROTATE_STEP)

    elif state == "DRIVING":
        if (time.time() - drive_start_time) < DRIVE_TIME:
            # Juster hvis ny aapning er bedre
            vinkel, avstand = vurder_aapning()
            if vinkel is not None:
                delta = ((vinkel - heading + 540) % 360) - 180
                if abs(delta) > 10:
                    omega = ROTATE_STEP if delta > 0 else -ROTATE_STEP
                    return (STEP, 0, omega)
            return (STEP, 0, 0.0)
        else:
            state = "IDLE"
            log.info("[HINDRING] Ferdig med kjore mot aapning, tilbake til IDLE.")
            return (STEP, 0, 0.0)

    else:
        state = "IDLE"
        return (0, 0, 0.0)
