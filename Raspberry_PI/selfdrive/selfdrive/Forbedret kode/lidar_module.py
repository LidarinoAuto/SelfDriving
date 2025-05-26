# -*- coding: utf-8 -*-
# Filename: lidar_module.py

from rplidar import RPLidar
import threading
import time
import math
from logging_utils import skriv_logg

PORT_NAME = "/dev/rplidar"
BAUD_RATE = 115200

current_distance_lidar_mm = 9999
lidar_buffer_mm = []
running = False
lidar = None
lidar_thread_instance = None

def start_lidar():
    global running, lidar, lidar_thread_instance
    skriv_logg(f"Attempting to start Lidar at {PORT_NAME} with baud rate {BAUD_RATE}...")
    stop_lidar()
    try:
        lidar = RPLidar(PORT_NAME, baudrate=BAUD_RATE)
        time.sleep(1)
        running = True
        lidar_thread_instance = threading.Thread(target=_lidar_thread, daemon=True)
        lidar_thread_instance.start()
        skriv_logg("Lidar initialized and data collection thread started.")
        return True
    except Exception as e:
        skriv_logg(f"Failed to initialize Lidar: {e}")
        lidar = None
        running = False
        lidar_thread_instance = None
        return False

def stop_lidar():
    global running, lidar, lidar_thread_instance
    skriv_logg("Attempting to stop Lidar...")
    running = False
    skriv_logg("Signal sent to Lidar thread to stop.")
    if lidar_thread_instance is not None and lidar_thread_instance.is_alive():
        skriv_logg("Waiting for Lidar thread to join...")
        lidar_thread_instance.join(timeout=2.0)
        if lidar_thread_instance.is_alive():
            skriv_logg("Warning: Lidar thread did not finish gracefully within timeout.")
        else:
            skriv_logg("Lidar thread joined successfully.")
        lidar_thread_instance = None
    if lidar:
        try:
            skriv_logg("Stopping Lidar motor...")
            lidar.stop()
            time.sleep(0.1)
            skriv_logg("Disconnecting Lidar serial port...")
            lidar.disconnect()
            skriv_logg("Lidar disconnected.")
        except Exception as e:
            skriv_logg(f"Error during Lidar physical stop or disconnect: {e}")
        lidar = None
    else:
        skriv_logg("Lidar object was not initialized.")

def _lidar_thread():
    global current_distance_lidar_mm, running, lidar_buffer_mm, lidar
    if lidar is None:
        skriv_logg("Lidar thread: Lidar object is None, cannot start scanning.")
        running = False
        return
    skriv_logg("Lidar thread: Started scanning loop.")
    scan_count = 0
    try:
        for scan in lidar.iter_scans():
            scan_count += 1
            #skriv_logg(f"Lidar thread: Received scan {scan_count} with {len(scan)} raw data points.")
            if not running:
                #skriv_logg(f"Lidar thread: Exiting scan loop as 'running' flag is False (after scan {scan_count}).")
                break
            forward_distances_mm = []
            debug_points_in_sector = 0
            debug_points_quality_zero = 0
            for m in scan:
                quality, angle, distance = m
                if quality == 0:
                    debug_points_quality_zero += 1
                    continue
                normalized_angle = angle if angle <= 180 else angle - 360
                if abs(normalized_angle) <= 30:
                    debug_points_in_sector += 1
                    forward_distances_mm.append(distance)
            #skriv_logg(f"Lidar thread: Scan {scan_count} - Points in sector: {debug_points_in_sector}, Points quality 0: {debug_points_quality_zero}, Points after filtering: {len(forward_distances_mm)}")
            if forward_distances_mm:
                min_distance_mm = min(forward_distances_mm)
                current_distance_lidar_mm = min_distance_mm
                lidar_buffer_mm.append(min_distance_mm)
                if len(lidar_buffer_mm) > 5:
                    lidar_buffer_mm.pop(0)
                #skriv_logg(f"Lidar thread: Scan {scan_count} - Min distance found in sector: {min_distance_mm} mm. Buffer size: {len(lidar_buffer_mm)}")
            #else:
                #skriv_logg(f"Lidar thread: Scan {scan_count} - No valid data points found in the forward sector.")
    except Exception as e:
        #skriv_logg(f"Exception caught in Lidar thread scan loop: {e}")
    #finally:
        #skriv_logg("Lidar thread: Finished execution.")
        running = False

def get_median_lidar_reading_cm():
    if not running or not lidar_buffer_mm:
        return float('inf')
    try:
        sorted_buffer = sorted(lidar_buffer_mm)
        n = len(sorted_buffer)
        if n == 0:
            return float('inf')
        mid = n // 2
        median_distance_mm = sorted_buffer[mid] if n % 2 == 1 else (sorted_buffer[mid - 1] + sorted_buffer[mid]) / 2
        median_distance_cm = median_distance_mm / 10.0
        if median_distance_cm <= 0:
            return float('inf')
        return median_distance_cm
    except Exception:
        return float('inf')

# if __name__ == "__main__":
#     skriv_logg("Running Lidar module isolation test...")
#     if start_lidar():
#         skriv_logg("Lidar started. Waiting for data...")
#         time.sleep(5)
#         skriv_logg(f"Current median reading (cm): {get_median_lidar_reading_cm()}")
#         skriv_logg("Stopping Lidar...")
#         stop_lidar()
#         skriv_logg("Test finished.")
#     else:
#         skriv_logg("Failed to start Lidar for isolation test.")
