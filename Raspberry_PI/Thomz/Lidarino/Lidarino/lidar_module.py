# Filnavn: lidar_module.py
# Modul for � hente og prosessere avstandsm�linger fra en RPLidar
from rplidar import RPLidar
import threading
import time

# --- Konfigurasjon ---
PORT_NAME = "/dev/rplidar"        # Juster porten etter ditt system
lidar = RPLidar(PORT_NAME, baudrate=115200)

# Buffer for glatting av m�linger
lidar_buffer = []
running = True  # Kontrollvariabel for tr�den

def lidar_thread():
    """
    Leser kontinuerlig data fra LIDAR og lagrer de n�rmeste m�lingene i en buffer.
    Begrenser m�lingene til �30� fra rett fram (0� eller 360�).
    """
    global lidar_buffer, running
    for scan in lidar.iter_scans():
        # Velg kun m�linger n�r front (�30� fra 0 eller 360)
        distances = [m[2] for m in scan if abs(m[1] - 0) <= 30 or abs(m[1] - 360) <= 30]
        if distances:
            # Legg til n�rmeste m�ling i buffer
            lidar_buffer.append(min(distances))
            if len(lidar_buffer) > 5:
                lidar_buffer.pop(0)
        if not running:
            break

def get_median_lidar_reading():
    """
    Returnerer median av de siste LIDAR-avstandene for � filtrere ut st�y.
    Returnerer ? dersom ingen data er tilgjengelig enn�.
    """
    if not lidar_buffer:
        return float('inf')
    sorted_buf = sorted(lidar_buffer)
    n = len(sorted_buf)
    mid = n // 2
    return sorted_buf[mid] if n % 2 else (sorted_buf[mid - 1] + sorted_buf[mid]) / 2

def start_lidar_thread():
    """Starter en bakgrunnstr�d for � samle inn LIDAR-data."""
    t = threading.Thread(target=lidar_thread, daemon=True)
    t.start()
