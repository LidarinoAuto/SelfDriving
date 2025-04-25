# lidar_visualisering.py
import pygame
import lidar
import math
import time

WIDTH = 600
HEIGHT = 600
CENTER = (WIDTH // 2, HEIGHT // 2)
SCALE = 0.5  # mm til pixel (eks. 1 mm = 0.5 pixel)

def polar_to_cartesian(angle_deg, distance_mm):
    angle_rad = math.radians(angle_deg)
    x = math.cos(angle_rad) * distance_mm * SCALE
    y = math.sin(angle_rad) * distance_mm * SCALE
    return int(CENTER[0] + x), int(CENTER[1] - y)

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("LIDAR Live Visning")
    clock = pygame.time.Clock()

    lidar.start_lidar()

    running = True
    while running:
        screen.fill((0, 0, 0))  # Svart bakgrunn
        pygame.draw.circle(screen, (0, 255, 0), CENTER, 5)  # Robot sentrum

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Tegne alle punktene vi får fra lidar
        # Bruk siste tilgjengelige scan
        latest_scan = lidar.lidar_buffer[-1:] if lidar.lidar_buffer else []

        if latest_scan:
            stable_distance = lidar.get_median_lidar_reading()

            # Her kan vi simulere 0 grader (rett frem) og noen vinkler rundt
            for angle in range(-30, 31, 5):  # fra -30° til 30° i 5° steg
                x, y = polar_to_cartesian(angle, stable_distance)
                pygame.draw.circle(screen, (255, 255, 255), (x, y), 3)

        pygame.display.update()
        clock.tick(10)

    lidar.stop_lidar()
    pygame.quit()

if __name__ == "__main__":
    main()
