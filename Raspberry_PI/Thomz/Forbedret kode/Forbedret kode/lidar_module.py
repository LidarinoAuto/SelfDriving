# -*- coding: utf-8 -*-
# Filename: lidar_module.py

from rplidar import RPLidar
import threading
import time
import math

PORT_NAME = "/dev/rplidar"
BAUD_RATE = 115200

current_distance_lidar_mm = 9999
lidar_buffer_mm = []
running = False
lidar = None
lidar_thread_instance = None

def start_lidar():
    global running, lidar, lidar_thread_instance
    print(f"Attempting to start Lidar at {PORT_NAME} with baud rate {BAUD_RATE}...")
    stop_lidar()
    try:
        lidar = RPLidar(PORT_NAME, baudrate=BAUD_RATE)
        time.sleep(1)
        running = True
        lidar_thread_instance = threading.Thread(target=_lidar_thread, daemon=True)
        lidar_thread_instance.start()
        print("Lidar initialized and data collection thread started.")
        return True
    except Exception as e:
        print(f"Failed to initialize Lidar: {e}")
        lidar = None
        running = False
        lidar_thread_instance = None
        return False

def stop_lidar():
    global running, lidar, lidar_thread_instance
    print("Attempting to stop Lidar...")
    running = False
    print("Signal sent to Lidar thread to stop.")
    if lidar_thread_instance is not None and lidar_thread_instance.is_alive():
        print("Waiting for Lidar thread to join...")
        lidar_thread_instance.join(timeout=2.0)
        if lidar_thread_instance.is_alive():
            print("Warning: Lidar thread did not finish gracefully within timeout.")
        else:
            print("Lidar thread joined successfully.")
        lidar_thread_instance = None
    if lidar:
        try:
            print("Stopping Lidar motor...")
            lidar.stop()
            time.sleep(0.1)
            print("Disconnecting Lidar serial port...")
            lidar.disconnect()
            print("Lidar disconnected.")
        except Exception as e:
            print(f"Error during Lidar physical stop or disconnect: {e}")
        lidar = None
    else:
        print("Lidar object was not initialized.")

def _lidar_thread():
    global current_distance_lidar_mm, running, lidar_buffer_mm, lidar
    if lidar is None:
        print("Lidar thread: Lidar object is None, cannot start scanning.")
        running = False
        return
    print("Lidar thread: Started scanning loop.")
    scan_count = 0
    try:
        for scan in lidar.iter_scans():
            scan_count += 1
            print(f"Lidar thread: Received scan {scan_count} with {len(scan)} raw data points.")
            if not running:
                print(f"Lidar thread: Exiting scan loop as 'running' flag is False (after scan {scan_count}).")
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
            print(f"Lidar thread: Scan {scan_count} - Points in sector: {debug_points_in_sector}, Points quality 0: {debug_points_quality_zero}, Points after filtering: {len(forward_distances_mm)}")
            if forward_distances_mm:
                min_distance_mm = min(forward_distances_mm)
                current_distance_lidar_mm = min_distance_mm
                lidar_buffer_mm.append(min_distance_mm)
                if len(lidar_buffer_mm) > 5:
                    lidar_buffer_mm.pop(0)
                print(f"Lidar thread: Scan {scan_count} - Min distance found in sector: {min_distance_mm} mm. Buffer size: {len(lidar_buffer_mm)}")
            else:
                print(f"Lidar thread: Scan {scan_count} - No valid data points found in the forward sector.")
    except Exception as e:
        print(f"Exception caught in Lidar thread scan loop: {e}")
    finally:
        print("Lidar thread: Finished execution.")
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
#     print("Running Lidar module isolation test...")
#     if start_lidar():
#         print("Lidar started. Waiting for data...")
#         time.sleep(5)
#         print(f"Current median reading (cm): {get_median_lidar_reading_cm()}")
#         print("Stopping Lidar...")
#         stop_lidar()
#         print("Test finished.")
#     else:
#         print("Failed to start Lidar for isolation test.")
