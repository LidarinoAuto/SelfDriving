import numpy as np
import math
from rplidar import RPLidar
from filterpy.kalman import KalmanFilter

MAP_SIZE_PIXELS = 500
MAP_SIZE_METERS = 10
obstacle_threshold = 500

kf = KalmanFilter(dim_x=4, dim_z=2)
kf.F = np.eye(4)
kf.H = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])
kf.P *= 1000


def process_lidar_scan(scan):
    """ Behandler Lidar-data for kartoppdatering og SLAM """
    occupancy_grid = np.zeros((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))
    for (_, angle_deg, dist_mm) in scan:
        if dist_mm > 0 and dist_mm < obstacle_threshold:
            angle = math.radians(360 - angle_deg)
            x = (dist_mm / 1000.0) * math.cos(angle)
            y = (dist_mm / 1000.0) * math.sin(angle)
            px, py = meters_to_pixels(x, y)
            if 0 <= px < MAP_SIZE_PIXELS and 0 <= py < MAP_SIZE_PIXELS:
                occupancy_grid[py, px] = 1
    return occupancy_grid