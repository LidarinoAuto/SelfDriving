from rplidar import RPLidar
import threading
import time

PORT_NAME = "/dev/rplidar"
lidar = RPLidar(PORT_NAME, baudrate=115200)
lidar_buffer = []
running = True

def lidar_thread():
    global lidar_buffer, running
    for scan in lidar.iter_scans():
        distances = [m[2] for m in scan if abs(m[1] - 0) <= 30 or abs(m[1] - 360) <= 30]
        if distances:
            lidar_buffer.append(min(distances))
            if len(lidar_buffer) > 5:
                lidar_buffer.pop(0)
        if not running:
            break

def get_median_lidar_reading():
    if not lidar_buffer:
        return float('inf')
    sorted_buf = sorted(lidar_buffer)
    n = len(sorted_buf)
    return sorted_buf[n // 2] if n % 2 else (sorted_buf[n // 2 - 1] + sorted_buf[n // 2]) / 2

def start_lidar_thread():
    t = threading.Thread(target=lidar_thread, daemon=True)
    t.start()