import pygame
import lidar
import ultrasound
import kompas
import mpu6050
import calibration
from motorsignal import send_movement_command
from heading import oppdater_heading
from visualisering import tegn_robot, tegn_lidar, tegn_ultrasound
from hindringslogikk import autonom_logikk
import time
import math

# Konstanter
STEP = 100
ROTATE = 15.0
WIDTH = 600
HEIGHT = 600
CENTER = (WIDTH // 2, HEIGHT // 2)
SCALE = 2.0
KOMPASS_JUSTERING = 270

# For LIDAR historikk
lidar_points = []
MAX_LIDAR_POINTS = 200

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

    # --- SENSOROPPSTART ---
    lidar.start_lidar()
    ultrasound.setup_ultrasound()
    kompas.setup_compass()
    mpu6050.setup_mpu6050()

    modus = "manuell"
    prev_command = (0, 0, 0.0)
    fused_heading = kompas.read_heading()
    if fused_heading != -1:
        fused_heading = (fused_heading + KOMPASS_JUSTERING) % 360
    else:
        fused_heading = 0

    last_time = time.time()

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

        ultrasound.update_ultrasound_readings()

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
            x, y, omega = autonom_logikk()

        current_command = (x, y, omega)
        if current_command != prev_command:
            send_movement_command(x, y, omega)
            prev_command = current_command

        # Oppdater heading
        fused_heading, last_time = oppdater_heading(fused_heading, last_time)

        # --- TEGNING ---
        tegn_robot(screen, fused_heading)

        for angle, distance in lidar.scan_data:
            if distance > 0:
                lidar_points.append((angle, distance))

        if len(lidar_points) > MAX_LIDAR_POINTS:
            lidar_points[:] = lidar_points[-MAX_LIDAR_POINTS:]

        tegn_lidar(screen, lidar_points, fused_heading)
        tegn_ultrasound(screen, ultrasound.sensor_distances, fused_heading)

        pygame.display.update()
        clock.tick(30)

    send_movement_command(0, 0, 0.0)
    lidar.stop_lidar()
    pygame.quit()

if __name__ == "__main__":
    main()
