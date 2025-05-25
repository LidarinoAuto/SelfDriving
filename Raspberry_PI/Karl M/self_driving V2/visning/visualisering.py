import pygame
import math

# Konstanter
WIDTH = 600
HEIGHT = 600
CENTER = (WIDTH // 2, HEIGHT // 2)
SCALE = 2.0  # 1 cm = 2 pixels

# Farger
LIDAR_NAER_FARGE = (0, 191, 255)   # Lys bl� (n�r)
LIDAR_MIDT_FARGE = (0, 255, 255)   # Turkis
LIDAR_LANGT_FARGE = (0, 255, 0)    # Gr�nn
ULTRALYD_NAER_FARGE = (255, 0, 0)  # R�d
ULTRALYD_LANGT_FARGE = (0, 255, 0) # Gr�nn
VALGT_AAPNING_FARGE = (255, 255, 0) # Gul pil for valgt �pning

# �pninger (kun valgt midtpunkt)
valgt_midt = None

def sett_aapninger(_, midtpunkt=None):
    """Ignorer liste, ta kun vare p� midtpunktet til valgt �pning."""
    global valgt_midt
    valgt_midt = midtpunkt

def polar_to_cartesian(angle_deg, distance_cm, robot_heading_deg=0):
    corrected_angle = -(angle_deg + robot_heading_deg)
    angle_rad = math.radians(corrected_angle)
    x = math.cos(angle_rad) * distance_cm * SCALE
    y = math.sin(angle_rad) * distance_cm * SCALE
    return int(CENTER[0] + x), int(CENTER[1] - y)

def tegn_robot_sentrum(screen):
    pygame.draw.circle(screen, (0, 255, 0), CENTER, 5)

def tegn_heading_pil(screen, heading, font, color=(0, 0, 255)):
    heading_rad = math.radians(-heading)
    arrow_length = 50
    end_x = int(CENTER[0] + math.cos(heading_rad) * arrow_length)
    end_y = int(CENTER[1] - math.sin(heading_rad) * arrow_length)
    pygame.draw.line(screen, color, CENTER, (end_x, end_y), 4)
    # 'N' p� toppen
    if color == (0, 0, 255):
        north_text = font.render('N', True, (255, 0, 0))
        screen.blit(north_text, (WIDTH // 2 - 10, 10))

def tegn_lidar(screen, lidar_points, fused_heading):
    for angle, distance in lidar_points:
        x, y = polar_to_cartesian(angle, distance / 10.0, fused_heading)
        if distance < 500:
            color = LIDAR_NAER_FARGE
        elif distance < 1500:
            color = LIDAR_MIDT_FARGE
        else:
            color = LIDAR_LANGT_FARGE
        pygame.draw.circle(screen, color, (x, y), 2)

def tegn_ultralyd(screen, sensor_distances, sensor_angles, fused_heading):
    for sensor, distance in sensor_distances.items():
        if distance > 0:
            angle = sensor_angles[sensor]
            x, y = polar_to_cartesian(angle, distance, fused_heading)
            color = ULTRALYD_NAER_FARGE if distance < 30 else ULTRALYD_LANGT_FARGE
            pygame.draw.line(screen, color, CENTER, (x, y), 3)

def tegn_aapninger(screen, fused_heading):
    """Tegn kun gul pil mot valgt �pning."""
    if valgt_midt is not None:
        midt_rad = math.radians(-(valgt_midt + fused_heading))
        end_x = int(CENTER[0] + math.cos(midt_rad) * 80)
        end_y = int(CENTER[1] - math.sin(midt_rad) * 80)
        pygame.draw.line(screen, VALGT_AAPNING_FARGE, CENTER, (end_x, end_y), 4)

# -------- Statusvisning --------
_status_text = ""

def sett_status(text):
    global _status_text
    _status_text = text

def tegn_status(screen, x=10, y=10, color=(255,255,255), fontsize=24):
    if not _status_text:
        return
    font = pygame.font.SysFont("Arial", fontsize)
    for i, line in enumerate(_status_text.split("\n")):
        txt = font.render(line, True, color)
        screen.blit(txt, (x, y + i * (fontsize + 2)))
