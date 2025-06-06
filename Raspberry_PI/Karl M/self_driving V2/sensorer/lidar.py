# lidar.py
from rplidar import RPLidar
import threading
import time

PORT_NAME = "/dev/rplidar"
STOP_DISTANCE_LIDAR = 250  # mm (trygg minimums-avstand)
ROBOT_RADIUS = 117.5       # mm (halv diameter på roboten)

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

scan_data = []  # (vinkel, avstand) tuples

def _lidar_thread():
    global current_distance_lidar, running, lidar_buffer, scan_data
    for scan in lidar.iter_scans():
        temp_scan = []
        for measurement in scan:
            angle = measurement[1]
            distance = measurement[2]
            temp_scan.append((angle, distance))
        scan_data = temp_scan  # Overwrite med siste komplette skanning

        # For eksisterende logikk (fremover-målinger)
        distances = [d for a, d in temp_scan if abs(a - 0) <= 30 or abs(a - 360) <= 30]
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
    stable_distance = get_median_lidar_reading()
    adjusted_distance = stable_distance - ROBOT_RADIUS
    return adjusted_distance > STOP_DISTANCE_LIDAR
