import time
import threading
import logging
import RPi.GPIO as GPIO
import serial
import math
import numpy as np

from sensors import setup_ultrasound, check_ultrasound, get_best_ultrasound_direction, read_all_ultrasound, init_lidar, lidar_thread, setup_compass, read_compass, get_cardinal_direction
from odometry import update_pose, get_pose_parameters
from motorstyring import send_command, rotate_by_angle, move_forward, stop_robot, move_forward_distance, choose_direction_and_rotate
from database import log_debug_data, close_client
from mapping import initialize_grid, update_occupancy_grid, save_occupancy_grid, get_pose_from_matrix


logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

STOP_DISTANCE_ULTRASOUND = 15  # cm
STOP_DISTANCE_LIDAR = 250      # mm

ESP_PORT = "/dev/ttyUSB0"
esp = serial.Serial(ESP_PORT, 115200, timeout=1)
time.sleep(2)

LIDAR_PORT = "/dev/ttyUSB1"
lidar = init_lidar(LIDAR_PORT)
current_distance_lidar = [9999]  # Delt mellom tr�der (liste med ett element)
running_flag = [True]

current_pose = np.eye(3)
bus = setup_compass()

lidar_thread_obj = threading.Thread(target=lidar_thread, args=(lidar, current_distance_lidar, running_flag))
lidar_thread_obj.daemon = True
lidar_thread_obj.start()

setup_ultrasound()

# Initialiser occupancy grid
occupancy_grid = initialize_grid()

last_log_time = time.time()
last_map_update = time.time()

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
        
        # Oppdater occupancy grid hvert 2. sekund basert p� en ny Lidar-skan:
        if time.time() - last_map_update >= 2.0:
            scan = next(lidar.iter_scans())  # Hent en skanning
            occupancy_grid = update_occupancy_grid(occupancy_grid, current_pose, scan)
            logging.info("Occupancy grid oppdatert.")
            last_map_update = time.time()
        
        # Logg debugdata til InfluxDB hvert sekund:
        if time.time() - last_log_time >= 1.0:
            debug_data = {}
            ultrasound_data = read_all_ultrasound()
            debug_data.update({f"ultra_{k}": v for k, v in ultrasound_data.items()})
            debug_data["LIDAR_distance_mm"] = current_distance_lidar[0]
            debug_data["Compass_heading_deg"] = compass_heading
            x, y, theta = get_pose_parameters(current_pose)
            debug_data["Pose_x_mm"] = x
            debug_data["Pose_y_mm"] = y
            debug_data["Pose_theta_deg"] = math.degrees(theta)
            log_debug_data(debug_data)
            last_log_time = time.time()
        if time.time() - last_map_update >= 2.0:
            scan = next(lidar.iter_scans())  # Hent en skanning
            occupancy_grid = update_occupancy_grid(occupancy_grid, current_pose, scan)
            save_occupancy_grid(occupancy_grid)  # Lagre kartet til disk
            logging.info("Occupancy grid oppdatert og lagret til disk.")
            last_map_update = time.time()
        
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
