import math
import numpy as np

def se2_transform(x, y, theta):
    """Returnerer en 3x3 homogen transformasjonsmatrise for posisjon (x, y) og orientering theta (i rad)."""
    return np.array([[math.cos(theta), -math.sin(theta), x],
                     [math.sin(theta),  math.cos(theta), y],
                     [0, 0, 1]])

def twist_to_transform(vx, vy, omega, dt):
    """
    Konverterer en twist (vx, vy, omega) og tidssteg dt til en SE2-transformasjon.
    vx og vy i mm/s, omega i rad/s.
    """
    if abs(omega) < 1e-6:
        dx = vx * dt
        dy = vy * dt
        dtheta = 0
    else:
        dtheta = omega * dt
        dx = (vx * math.sin(dtheta) + vy * (1 - math.cos(dtheta))) / omega
        dy = (vy * math.sin(dtheta) - vx * (1 - math.cos(dtheta))) / omega
    return se2_transform(dx, dy, dtheta)

def update_pose(current_pose, vx, vy, omega, dt):
    """
    Oppdaterer pose basert p� twist og tidssteg dt.
    current_pose er en 3x3 matrise.
    Returnerer den nye pose.
    """
    increment = twist_to_transform(vx, vy, omega, dt)
    new_pose = current_pose @ increment
    return new_pose

def get_pose_parameters(pose):
    """Ekstraherer x, y og theta fra en SE2-matrise."""
    x = pose[0, 2]
    y = pose[1, 2]
    theta = math.atan2(pose[1, 0], pose[0, 0])
    return x, y, theta
