# lidar_visualisering.py
import pygame
import lidar
import ultrasound
import math
import time

WIDTH = 600
HEIGHT = 600
CENTER = (WIDTH // 2, HEIGHT // 2)
SCALE = 2.0  # Skaler litt opp for bedre synlighet (1 cm = 2 px)

def polar_to_cartesian(angle_deg, distance_cm):
    angle_rad = math.radians(-angle_deg)
    x = math.cos(angle_rad) * distance_cm * SCALE
    y = math.sin(angle_rad) * distance_cm * SCALE
    return int(CENTER[0] + x), int(CENTER[1] - y)

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("LIDAR + Ultralyd Live Visning")
    clock = pygame.time.Clock()

    lidar.start_lidar()
    ultrasound.setup_ultrasound()

    running = True
    while running:
        screen.fill((0, 0, 0))  # Svart bakgrunn
        pygame.draw.circle(screen, (0, 255, 0), CENTER, 5)  # Robot sentrum

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Tegn LIDAR
        for angle, distance in lidar.scan_data:
            if distance > 0:
                x, y = polar_to_cartesian(angle, distance / 10.0)  # mm til cm
                pygame.draw.circle(screen, (255, 255, 255), (x, y), 2)

        # Oppdater ultralydmålinger
        ultrasound.update_ultrasound_readings()

        # Tegn ultralyd-sensoravlesninger
        for sensor, distance in ultrasound.sensor_distances.items():
            if distance > 0:
                angle = ultrasound.sensor_angles[sensor]
                x, y = polar_to_cartesian(angle, distance)
                color = (255, 0, 0) if distance < 30 else (0, 255, 0)
                pygame.draw.line(screen, color, CENTER, (x, y), 3)

        pygame.display.update()
        clock.tick(10)

    lidar.stop_lidar()
    pygame.quit()

if __name__ == "__main__":
    main()
