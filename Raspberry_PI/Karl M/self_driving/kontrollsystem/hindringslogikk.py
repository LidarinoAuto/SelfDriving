# hindringslogikk.py
from sensorer import lidar
from sensorer import ultrasound

STEP = 100  # Flyttesteg


def autonom_logikk():
    """
    Enkel hindringslogikk:
    - Hvis det er fri vei foran (LIDAR) og ingen ultralyd-hindringer, kjør fremover.
    - Hvis ikke, stopp.
    """
    path_clear = lidar.is_path_clear()

    front_blocked = False
    if ultrasound.sensor_distances["front_left"] > 0 and ultrasound.sensor_distances["front_left"] < 30:
        front_blocked = True
    if ultrasound.sensor_distances["front_right"] > 0 and ultrasound.sensor_distances["front_right"] < 30:
        front_blocked = True

    if path_clear and not front_blocked:
        return (STEP, 0, 0.0)
    else:
        return (0, 0, 0.0)
