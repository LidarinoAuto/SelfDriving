# kontroll.py
import pygame
from motorsignal import send_movement_command
import lidar
import time

STEP = 100
ROTATE = 45.5

def autonom_logikk():
    if lidar.is_path_clear():
        return (STEP, 0, 0.0)  # Kjør fremover
    else:
        return (0, 0, 0.0)      # Stopp

def main():
    pygame.init()
    screen = pygame.display.set_mode((400, 200))
    pygame.display.set_caption("Robotkontroll (Manuell/Autonom)")
    clock = pygame.time.Clock()

    x = y = omega = 0
    prev_command = (0, 0, 0.0)
    modus = "manuell"

    lidar.start_lidar()

    running = True
    while running:
        x = y = omega = 0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_TAB:
                    modus = "autonom" if modus == "manuell" else "manuell"
                    print(f"Byttet til: {modus.upper()}")
                    time.sleep(0.2)

        keys = pygame.key.get_pressed()

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

        clock.tick(20)

    send_movement_command(0, 0, 0.0)
    lidar.stop_lidar()
    pygame.quit()

if __name__ == "__main__":
    main()
