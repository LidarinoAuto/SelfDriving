# kontroll.py
import pygame
import lidar
import ultrasound
import kompas
from motorsignal import send_movement_command
import math
import time

# Konstanter
STEP = 100  # mm/s
ROTATE = 45.5  # grader/s
WIDTH = 600
HEIGHT = 600
CENTER = (WIDTH // 2, HEIGHT // 2)
SCALE = 2.0  # 1 cm = 2 pixels for visualisering

KOMPASS_JUSTERING = 0  # Endre til 90, 180 eller 270 om kompasset peker feil

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
        return (STEP, 0, 0.0)  # Kjør fremover
    else:
        return (0, 0, 0.0)     # Stopp

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Robotkontroll + Kart + Kompass")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 24)

    x = y = omega = 0
    prev_command = (0, 0, 0.0)
    modus = "manuell"

    lidar.start_lidar()
    ultrasound.setup_ultrasound()
    kompas.setup_compass()

    running = True
    while running:
        screen.fill((0, 0, 0))  # Svart bakgrunn

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

        # --- Tegn kartet ---
        # Tegn robotens sentrum
        pygame.draw.circle(screen, (0, 255, 0), CENTER, 5)

        # Tegn kompasspil
        heading = kompas.read_heading()
        if heading != -1:
            heading = (heading + KOMPASS_JUSTERING) % 360
            heading_rad = math.radians(-heading)
            arrow_length = 50
            end_x = int(CENTER[0] + math.cos(heading_rad) * arrow_length)
            end_y = int(CENTER[1] - math.sin(heading_rad) * arrow_length)
            pygame.draw.line(screen, (0, 0, 255), CENTER, (end_x, end_y), 4)  # Blå pil for heading

        # Tegn 'N' (Nord) øverst
        north_text = font.render('N', True, (255, 0, 0))
        screen.blit(north_text, (WIDTH//2 - 10, 10))

        # Tegn LIDAR-målinger
        for angle, distance in lidar.scan_data:
            if distance > 0:
                x_l, y_l = polar_to_cartesian(angle, distance / 10.0)  # mm -> cm
                pygame.draw.circle(screen, (255, 255, 255), (x_l, y_l), 2)

        # Tegn ultralydsensor-målinger
        for sensor, distance in ultrasound.sensor_distances.items():
            if distance > 0:
                angle = ultrasound.sensor_angles[sensor]
                x_u, y_u = polar_to_cartesian(angle, distance)
                color = (255, 0, 0) if distance < 30 else (0, 255, 0)
                pygame.draw.line(screen, color, CENTER, (x_u, y_u), 3)

        pygame.display.update()
        clock.tick(20)

    # --- Avslutt ---
    send_movement_command(0, 0, 0.0)
    lidar.stop_lidar()
    pygame.quit()

if __name__ == "__main__":
    main()
