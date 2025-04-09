import time
import math
import logging
from odometry import update_pose, get_pose_parameters

# Parametere (juster etter behov)
SPEED = 100  # mm/s
TIME_TO_MOVE_SHORT = 10 / SPEED  # Tid for 10 mm bevegelse (sekunder)
ROTATION_SPEED = 2  # Kommandoparameter til ESP32
ROTATION_TIME_90_DEG = 1.5  # Tid for 90� rotasjon

def send_command(esp, command):
    logging.info(f"Sender til ESP32: {command.strip()}")
    esp.write(command.encode())

def rotate_by_angle(esp, angle, current_pose):
    """
    Roterer roboten med angitt vinkel (grader).
    Beregner rotasjonstiden proporsjonalt med 90�,
    og oppdaterer den interne pose. Returnerer ny pose.
    """
    rotation_time = (abs(angle) / 90) * ROTATION_TIME_90_DEG
    omega = math.radians(angle) / rotation_time  # rad/s
    if angle < 0:
        send_command(esp, f"0 0 {ROTATION_SPEED}\n")
    else:
        send_command(esp, f"0 0 {-ROTATION_SPEED}\n")
    time.sleep(rotation_time)
    send_command(esp, "0 0 0\n")
    new_pose = update_pose(current_pose, 0, 0, omega, rotation_time)
    x, y, theta = get_pose_parameters(new_pose)
    logging.info(f"Oppdatert pose etter rotasjon: x={x:.1f} mm, y={y:.1f} mm, theta={math.degrees(theta):.1f}�")
    return new_pose

def move_forward(esp):
    send_command(esp, "100 0 0\n")

def stop_robot(esp):
    send_command(esp, "0 0 0\n")

def move_forward_distance(esp, current_pose):
    send_command(esp, f"{SPEED} 0 0\n")
    time.sleep(TIME_TO_MOVE_SHORT)
    send_command(esp, "0 0 0\n")
    new_pose = update_pose(current_pose, SPEED, 0, 0, TIME_TO_MOVE_SHORT)
    return new_pose

def choose_direction_and_rotate(esp, best_sensor, current_pose):
    if best_sensor == "FrontVenstre":
        angle = -30
    elif best_sensor == "FrontHoyre":
        angle = 30
    elif best_sensor == "BakVenstre":
        angle = -150
    elif best_sensor == "BakHoyre":
        angle = 150
    else:
        angle = 0
    logging.info(f"Roterer basert p� sensor: {best_sensor}, vinkel: {angle}�")
    new_pose = rotate_by_angle(esp, angle, current_pose)
    return new_pose
