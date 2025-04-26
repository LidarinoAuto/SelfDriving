# kontroll.py
import pygame
import lidar
import ultrasound
import kompas
import mpu6050
import calibration
from motorsignal import send_movement_command
import math
import time
import os

# Konstanter
STEP = 100
ROTATE = 15.0
WIDTH = 600
HEIGHT = 600
CENTER = (WIDTH // 2, HEIGHT // 2)
SCALE = 2.0  # 1 cm = 2 pixels
KOMPASS_JUSTERING = 270  # Hvis nødvendig

# Filter-konstanter
GYRO_WEIGHT = 0.98
COMPASS_WEIGHT = 1.0 - GYRO_WEIGHT

def polar_to_cartesian(angle_deg, distance_cm):
    angle_rad = math.radians(-angle_deg)
    x = math.cos(angle_rad) * distance_cm * SCALE
    y = math.sin(angle_rad) * distance_cm * SCALE
    return int(CENTER[0] + x), int(CENTER[1] - y)

def autonom_logikk():
    path_clear = lidar.is_path_clear()

    front_blocked = False
    if ultrasound.sensor_distances["front_left"] > 0 and ultrasound.sensor_distances["front_left"] < 30:
        front_blocked = True
    if ultrasound.sensor_distances["front_right"] > 0 and ultrasound.sensor_distances["front_right"] < 30:
        front_blocked = True

    if path_clear and not front_blocked:
        return (STEP, 0, 0.0)
    else:
        return (0, 0, 0.0)

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Robotkontroll + Kart + Kalibrering")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 24)

    # --- KALIBRERING ---
    screen.fill((0, 0, 0))
    text = font.render('Kalibrerer gyro...', True, (255, 255, 0))
    screen.blit(text, (150, 250))
    pygame.display.update()
    calibration.calibrate_gyro()

    screen.fill((0, 0, 0))
    text = font.render('Kalibrerer kompass...', True, (255, 255, 0))
    screen.blit(text, (150, 250))
    pygame.display.update()
    calibration.calibrate_compass()

    screen.fill((0, 0, 0))
    text = font.render('Kalibrering ferdig!', True, (0, 255, 0))
    screen.blit(text, (150, 250))
    pygame.display.update()
    time.sleep(2)

    # --- NORMAL DRIFT ---
    lidar.start_lidar()
    ultrasound.setup_ultrasound()
    kompas.setup_compass()
    mpu6050.setup_mpu6050()

    last_time = time.time()
    prev_command = (0, 0, 0.0)

    # Start med heading fra kompass
    fused_heading = kompas.read_heading()
    if fused_heading != -1:
        fused_heading = (fused_heading + KOMPASS_JUSTERING) % 360
    else:
        fused_heading = 0

    modus = "manuell"
    running = True
    while running:
        screen.fill((0, 0, 0))

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_TAB:
                    modus = "autonom" if modus == "manuell" else "manuell"
                    print(f"Byttet til: {modus.upper()}")
                    time.sleep(0.2)

        keys = pygame.key.get_pressed()
        x = y = omega = 0

        if modus == "manuell":
            if keys[pygame.K_w]:
                x = STEP
            elif keys[pygame.K_s]:
                x = -STEP
            if keys[pygame.K_a]:
                y = STEP
            elif keys[pygame.K_d]:
                y = -STEP
            if keys[pygame.K_q]:
                omega = ROTATE
            elif keys[pygame.K_e]:
                omega = -ROTATE

        elif modus == "autonom":
            ultrasound.update_ultrasound_readings()
            x, y, omega = autonom_logikk()

        current_command = (x, y, omega)

        if current_command != prev_command:
            send_movement_command(x, y, omega)
            prev_command = current_command

        # --- Oppdater heading ---
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        gyro_z = mpu6050.read_gyro_z()
        fused_heading += gyro_z * dt
        fused_heading %= 360

        compass_heading = kompas.read_heading()
        if compass_heading != -1:
            compass_heading = (compass_heading + KOMPASS_JUSTERING) % 360
            fused_heading = (GYRO_WEIGHT * fused_heading + COMPASS_WEIGHT * compass_heading) % 360

        # --- Tegning ---
        pygame.draw.circle(screen, (0, 255, 0), CENTER, 5)

        heading_rad = math.radians(-fused_heading)
        arrow_length = 50
        end_x = int(CENTER[0] + math.cos(heading_rad) * arrow_length)
        end_y = int(CENTER[1] - math.sin(heading_rad) * arrow_length)
        pygame.draw.line(screen, (0, 0, 255), CENTER, (end_x, end_y), 4)

        north_text = font.render('N', True, (255, 0, 0))
        screen.blit(north_text, (WIDTH//2 - 10, 10))

        # Tegn LIDAR
        for angle, distance in lidar.scan_data:
            if distance > 0:
                x, y = polar_to_cartesian(angle - fused_heading, distance / 10.0)
                pygame.draw.circle(screen, (255, 255, 255), (x, y), 2)

        
        # Tegn ultralyd-sensoravlesninger
        for sensor, distance in ultrasound.sensor_distances.items():
            if distance > 0:
                angle = ultrasound.sensor_angles[sensor]
                x, y = polar_to_cartesian(angle - fused_heading, distance)
                color = (255, 0, 0) if distance < 30 else (0, 255, 0)
                pygame.draw.line(screen, color, CENTER, (x, y), 3)




        pygame.display.update()
        clock.tick(30)

    send_movement_command(0, 0, 0.0)
    lidar.stop_lidar()
    pygame.quit()

if __name__ == "__main__":
    main()
