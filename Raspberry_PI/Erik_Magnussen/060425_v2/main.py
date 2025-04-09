#!/usr/bin/env python3
import time
import threading
import logging
import RPi.GPIO as GPIO
import serial
import math
import numpy as np
import os
import queue

from sensors import (
    setup_ultrasound, check_ultrasound, get_best_ultrasound_direction,
    read_all_ultrasound, init_lidar, lidar_thread, setup_compass, read_compass,
    get_cardinal_direction
)
from odometry import update_pose, get_pose_parameters
from motorstyring import (
    send_command, rotate_by_angle, move_forward, stop_robot,
    move_forward_distance, choose_direction_and_rotate
)
from database import log_debug_data, close_client
from mapping import initialize_grid, update_occupancy_grid, save_occupancy_grid
from webserver import app  # Flask-appen

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Konstanter
STOP_DISTANCE_ULTRASOUND = 15  # cm
STOP_DISTANCE_LIDAR = 250      # mm

# Oppsett for ESP32 (via serial)
ESP_PORT = "/dev/ttyUSB0"
try:
    esp = serial.Serial(ESP_PORT, 115200, timeout=1)
except Exception as e:
    logging.error(f"Feil ved oppsett av ESP32: {e}")
    raise e
time.sleep(2)

# Oppsett for LIDAR
LIDAR_PORT = "/dev/ttyUSB1"
lidar = init_lidar(LIDAR_PORT)
current_distance_lidar = [9999]  # Delt mellom tr�der
running_flag = [True]

# Intern pose (SE2, 3x3 identitetsmatrise)
current_pose = np.eye(3)

# Sett opp kompass
bus = setup_compass()

# Start LIDAR-tr�den
lidar_thread_obj = threading.Thread(target=lidar_thread, args=(lidar, current_distance_lidar, running_flag))
lidar_thread_obj.daemon = True
lidar_thread_obj.start()

# Sett opp ultralyd
setup_ultrasound()

# Initialiser occupancy grid
occupancy_grid = initialize_grid()

# Start webserver i en egen tr�d (bruker port 5001, velg en ledig port)
def start_webserver():
    app.run(host="0.0.0.0", port=5001, debug=False, use_reloader=False)

web_thread = threading.Thread(target=start_webserver)
web_thread.daemon = True
web_thread.start()

last_log_time = time.time()
last_map_update = time.time()

# BASE_DIR for lagring (skal v�re identisk med det som brukes i mapping.py og webserver.py)
BASE_DIR = "/home/admin/GitHub/SelfDriving/Raspberry_PI/Erik_Magnussen/060425_v2"
IMAGE_PATH = os.path.join(BASE_DIR, "occupancy_grid.png")

try:
    move_forward(esp)
    while True:
        compass_heading = read_compass(bus)
        cardinal = get_cardinal_direction(compass_heading)
        logging.info(f"Kompassretning: {compass_heading:.2f}� ({cardinal})")

        if check_ultrasound(STOP_DISTANCE_ULTRASOUND):
            stop_robot(esp)
            time.sleep(2)
            best_sensor, best_distance = get_best_ultrasound_direction()
            current_pose = choose_direction_and_rotate(esp, best_sensor, current_pose)
            current_pose = move_forward_distance(esp, current_pose)
            move_forward(esp)
            continue

        logging.info(f"LIDAR-avstand: {current_distance_lidar[0]:.1f} mm")
        if current_distance_lidar[0] <= STOP_DISTANCE_LIDAR:
            stop_robot(esp)
            time.sleep(2)
            best_sensor, best_distance = get_best_ultrasound_direction()
            current_pose = choose_direction_and_rotate(esp, best_sensor, current_pose)
            current_pose = move_forward_distance(esp, current_pose)
            move_forward(esp)

        if time.time() - last_map_update >= 2.0:
            try:
                scan = next(lidar.iter_scans())
                occupancy_grid = update_occupancy_grid(occupancy_grid, current_pose, scan)
                save_occupancy_grid(occupancy_grid)
                if os.path.exists(IMAGE_PATH):
                    logging.info(f"Occupancy grid oppdatert og lagret til disk: {IMAGE_PATH}")
                else:
                    logging.error(f"Occupancy grid ble ikke lagret til: {IMAGE_PATH}")
            except Exception as e:
                logging.error(f"Feil ved oppdatering av occupancy grid: {e}")
            last_map_update = time.time()

        if time.time() - last_log_time >= 1.0:
            debug_data = {}
            ultrasound_data = read_all_ultrasound()
            debug_data.update({f"ultra_{k}": v for k, v in ultrasound_data.items()})
            debug_data["LIDAR_distance_mm"] = float(current_distance_lidar[0])
            debug_data["Compass_heading_deg"] = compass_heading
            x, y, theta = get_pose_parameters(current_pose)
            debug_data["Pose_x_mm"] = x
            debug_data["Pose_y_mm"] = y
            debug_data["Pose_theta_deg"] = math.degrees(theta)
            log_debug_data(debug_data)
            last_log_time = time.time()

        time.sleep(0.1)

except KeyboardInterrupt:
    logging.info("Avslutter programmet...")
    stop_robot(esp)

finally:
    running_flag[0] = False
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    esp.close()
    GPIO.cleanup()
    close_client()
    logging.info("Systemet er ryddet opp.")
