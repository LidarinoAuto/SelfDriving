import pygame
from motorsignal import send_movement_command

STEP = 100
ROTATE = 25.0

def main():
    pygame.init()
    screen = pygame.display.set_mode((300, 200))
    pygame.display.set_caption("Robotkontroll")
    clock = pygame.time.Clock()

    x = y = omega = 0
    prev_command = (0, 0, 0.0)

    running = True
    while running:
        x = y = omega = 0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        keys = pygame.key.get_pressed()

        # X = frem/bak
        if keys[pygame.K_w]:
            x = STEP
        elif keys[pygame.K_s]:
            x = -STEP

        # Y = sideveis (n� riktig vei)
        if keys[pygame.K_a]:
            y = STEP       # venstre
        elif keys[pygame.K_d]:
            y = -STEP      # h�yre

        if keys[pygame.K_q]:
            omega = ROTATE
        elif keys[pygame.K_e]:
            omega = -ROTATE

        current_command = (x, y, omega)

        if current_command != prev_command:
            send_movement_command(x, y, omega)
            prev_command = current_command

        clock.tick(20)

    send_movement_command(0, 0, 0.0)
    pygame.quit()

if __name__ == "__main__":
    main()
