import numpy as np
import math
import matplotlib.pyplot as plt
import os
import logging

# BASE_DIR satt til �nsket bane
BASE_DIR = "/home/admin/GitHub/SelfDriving/Raspberry_PI/Erik_Magnussen/060425_v2"

CELL_SIZE = 10        # hver celle representerer 10 mm
GRID_SIZE = 500       # 500x500 celler (ca. 5x5 m)
ORIGIN = (GRID_SIZE // 2, GRID_SIZE // 2)  # Startposisjon i midten

def initialize_grid():
    """
    Oppretter et occupancy grid:
      - -1 = ukjent,
      - 0  = fri,
      - 1  = opptatt
    """
    return -np.ones((GRID_SIZE, GRID_SIZE), dtype=int)

def world_to_grid(x, y):
    grid_x = int(x / CELL_SIZE) + ORIGIN[0]
    grid_y = int(y / CELL_SIZE) + ORIGIN[1]
    return grid_x, grid_y

def bresenham_line(x0, y0, x1, y1):
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            cells.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            cells.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    cells.append((x1, y1))
    return cells

def update_occupancy_grid(grid, current_pose, scan):
    robot_x, robot_y, _ = get_pose_from_matrix(current_pose)
    robot_grid = world_to_grid(robot_x, robot_y)
    
    for meas in scan:
        quality, angle_deg, distance = meas
        if distance <= 0:
            continue
        angle_rad = math.radians(angle_deg)
        x_r = distance * math.cos(angle_rad)
        y_r = distance * math.sin(angle_rad)
        point_robot = np.array([x_r, y_r, 1])
        point_global = current_pose @ point_robot
        gx, gy = point_global[0], point_global[1]
        grid_cell = world_to_grid(gx, gy)
        line_cells = bresenham_line(robot_grid[0], robot_grid[1], grid_cell[0], grid_cell[1])
        for cell in line_cells[:-1]:
            if 0 <= cell[0] < grid.shape[0] and 0 <= cell[1] < grid.shape[1]:
                grid[cell[0], cell[1]] = 0
        if 0 <= grid_cell[0] < grid.shape[0] and 0 <= grid_cell[1] < grid.shape[1]:
            grid[grid_cell[0], grid_cell[1]] = 1
    return grid

def get_pose_from_matrix(pose):
    x = pose[0, 2]
    y = pose[1, 2]
    theta = math.atan2(pose[1, 0], pose[0, 0])
    return x, y, theta

def save_occupancy_grid(grid, filename="occupancy_grid.png"):
    filepath = os.path.join(BASE_DIR, filename)
    # Fjern gammel fil hvis den finnes
    if os.path.exists(filepath):
        os.remove(filepath)
    plt.figure(figsize=(6,6))
    plt.imshow(grid, cmap='gray', origin='lower')
    plt.colorbar(label='Occupancy')
    plt.title("Occupancy Grid")
    plt.xlabel("X-celle")
    plt.ylabel("Y-celle")
    plt.savefig(filepath)
    plt.close()
    logging.info(f"Occupancy grid lagret til: {filepath}")
