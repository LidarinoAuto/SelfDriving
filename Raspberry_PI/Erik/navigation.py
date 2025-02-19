import math
from simple_pid import PID
from dynamic_astar import DynamicAStar

astar_planner = DynamicAStar()

pid_x = PID(1.0, 0.1, 0.05, setpoint=0)
pid_y = PID(1.0, 0.1, 0.05, setpoint=0)

def compute_path(robot_pos, target_pos, grid):
    """ Dynamisk A* eller RRT for path planning """
    return astar_planner.plan(grid, robot_pos, target_pos)

def navigate_with_pid(robot_pos, target_pos):
    """ Bruk PID for å justere robotens bevegelse """
    error_x = target_pos[0] - robot_pos[0]
    error_y = target_pos[1] - robot_pos[1]

    vx = pid_x(error_x)
    vy = pid_y(error_y)

    send_speed(vx, vy, 0)