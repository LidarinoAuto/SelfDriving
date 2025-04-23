# kontroll.py
import curses
from motorsignal import send_movement_command

STEP = 100     # mm/s
ROTATE = 30    # grader/s

def main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.clear()
    
    x = 0
    y = 0
    omega = 0

    stdscr.addstr(0, 0, "Bruk piltaster til å styre. Trykk q for å avslutte.")
    
    while True:
        key = stdscr.getch()
        
        if key == curses.KEY_UP:
            y = STEP
        elif key == curses.KEY_DOWN:
            y = -STEP
        elif key == curses.KEY_LEFT:
            x = -STEP
        elif key == curses.KEY_RIGHT:
            x = STEP
        elif key == ord('a'):
            omega = ROTATE
        elif key == ord('d'):
            omega = -ROTATE
        elif key == ord(' '):  # Stopp
            x = y = omega = 0
        elif key == ord('q'):
            break
        else:
            continue

        send_movement_command(x, y, omega)
        stdscr.addstr(2, 0, f"x: {x}, y: {y}, omega: {omega}       ")

if __name__ == "__main__":
    curses.wrapper(main)
