# kontroll.py
import pygame
from sensorer import lidar
from sensorer import ultrasound
from sensorer import kompas
from sensorer import mpu6050
from kontrollsystem import calibration
from kontrollsystem.motorsignal import send_movement_command
from kontrollsystem.heading import HeadingTracker
from visning.visualisering import tegn_robot_sentrum, tegn_heading_pil, tegn_lidar, tegn_ultralyd
from kontrollsystem.hindringslogikk import autonom_logikk
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
    heading_tracker = HeadingTracker()
    heading_tracker.setup()

    modus = "manuell"
    prev_command = (0, 0, 0.0)

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
            x, y, omega = autonom_logikk(heading_tracker.get_heading())

        current_command = (x, y, omega)
        if current_command != prev_command:
            send_movement_command(x, y, omega)
            prev_command = current_command

        # Oppdater heading
        fused_heading = heading_tracker.update()

        # --- TEGNING ---
        tegn_robot_sentrum(screen)
        tegn_heading_pil(screen, fused_heading, font)

        for angle, distance in lidar.scan_data:
            if distance > 0:
                lidar_points.append((angle, distance))

        if len(lidar_points) > MAX_LIDAR_POINTS:
            lidar_points[:] = lidar_points[-MAX_LIDAR_POINTS:]

        tegn_lidar(screen, lidar_points, fused_heading)
        tegn_ultralyd(screen, ultrasound.sensor_distances, ultrasound.sensor_angles, fused_heading)

        pygame.display.update()
        clock.tick(30)

    send_movement_command(0, 0, 0.0)
    lidar.stop_lidar()
    pygame.quit()

if __name__ == "__main__":
    main()
