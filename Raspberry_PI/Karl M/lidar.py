# lidar.py
from rplidar import RPLidar
import threading
import time

PORT_NAME = "/dev/rplidar"
STOP_DISTANCE_LIDAR = 250  # mm

# Globale variabler
current_distance_lidar = 9999
lidar_buffer = []
running = False
lidar = None

def start_lidar():
    global running, lidar
    lidar = RPLidar(PORT_NAME, baudrate=115200)
    running = True
    threading.Thread(target=_lidar_thread, daemon=True).start()

def stop_lidar():
    global running, lidar
    running = False
    if lidar:
        lidar.stop()
        lidar.disconnect()

def _lidar_thread():
    global current_distance_lidar, running, lidar_buffer
    for scan in lidar.iter_scans():
        distances = [measurement[2] for measurement in scan
                     if abs(measurement[1] - 0) <= 30 or abs(measurement[1] - 360) <= 30]
        if distances:
            min_distance = min(distances)
            current_distance_lidar = min_distance
            lidar_buffer.append(min_distance)
            if len(lidar_buffer) > 5:
                lidar_buffer.pop(0)
        if not running:
            break

def get_median_lidar_reading():
    if not lidar_buffer:
        return float('inf')
    sorted_buffer = sorted(lidar_buffer)
    n = len(sorted_buffer)
    if n % 2 == 1:
        return sorted_buffer[n // 2]
    else:
        return (sorted_buffer[n // 2 - 1] + sorted_buffer[n // 2]) / 2

def is_path_clear():
    """
    Returnerer True hvis avstanden foran er større enn STOP_DISTANCE_LIDAR.
    """
    stable_distance = get_median_lidar_reading()
    return stable_distance > STOP_DISTANCE_LIDAR
