import threading
import pygame
import sys
from rplidar import RPLidar
from lidar_processing import process_lidar_scan
from navigation import compute_path, navigate_with_pid
from web_server import run_flask

LIDAR_PORT = '/dev/ttyUSB0'
robot_position = [0, 0]
target_position = None


def main():
    """ Hovedløkke for navigasjon, SLAM og kontroll """
    global robot_position, target_position
    
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.daemon = True
    flask_thread.start()
    
    lidar = RPLidar(LIDAR_PORT)
    lidar.start_motor()
    running = True
    
    try:
        scan_iterator = lidar.iter_scans()
        while running:
            scan = next(scan_iterator)
            occupancy_grid = process_lidar_scan(scan)
            
            if target_position:
                robot_position = navigate_with_pid(robot_position, target_position)
            
            pygame.display.flip()
    
    except KeyboardInterrupt:
        lidar.stop_motor()
        lidar.disconnect()
        pygame.quit()
        sys.exit(0)

if __name__ == "__main__":
    main()