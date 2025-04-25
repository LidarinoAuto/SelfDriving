# lidar_visualisering.py
import pygame
import lidar
import math
import time

WIDTH = 600
HEIGHT = 600
CENTER = (WIDTH // 2, HEIGHT // 2)
SCALE = 0.5  # mm til pixel

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

        # Tegne ALLE målinger
        for angle, distance in lidar.scan_data:
            if distance > 0:  # Noen LIDAR gir 0 på ugyldige målinger
                x, y = polar_to_cartesian(angle, distance)
                pygame.draw.circle(screen, (255, 255, 255), (x, y), 2)

        pygame.display.update()
        clock.tick(20)

    lidar.stop_lidar()
    pygame.quit()

if __name__ == "__main__":
    main()
