#!/usr/bin/env python3
import serial
import time
import curses
import logging

# Konfigurer logging
logging.basicConfig(level=logging.INFO)

# ----------------- KONSTANTER -----------------
ESP_PORT = "/dev/ttyUSB0"          # Port til ESP32
SERIAL_BAUDRATE = 115200

SPEED = 100                        # mm/s (brukes for fremover/bakover)
ROTATION_SPEED = 2                 # Rotasjonshastighet (brukes for rotasjon)

# ----------------- MOTORSTYRING -----------------
class MotorController:
    def __init__(self, esp_port=ESP_PORT, baudrate=SERIAL_BAUDRATE):
        try:
            self.esp = serial.Serial(esp_port, baudrate, timeout=1)
            time.sleep(2)  # Gi ESP32 tid til oppstart
            logging.info("MotorController initialisert.")
        except Exception as e:
            logging.error(f"Feil ved åpning av seriell port: {e}")

    def send_command(self, x, y, r):
        command = f"{x} {y} {r}\n"
        logging.info(f"Sender kommando: {command.strip()}")
        self.esp.write(command.encode())

    def close(self):
        self.esp.close()
        logging.info("Serial port lukket.")

# ----------------- MANUELL KONTROLL VIA TASTATURET -----------------
def keyboard_control_mode(motor):
    """
    Leser tastetrykk og sender motorkommandoer til roboten.
    W - Fremover, S - Bakover, E - Roter venstre, Q - Roter høyre,
    X - Stopp, ESC - Avslutt.
    """
    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.clear()
    
    stdscr.addstr(0, 0, "Manuell kontrollmodus for roboten:\n")
    stdscr.addstr("W: Fremover\nS: Bakover\nE: Roter venstre\nQ: Roter høyre\nX: Stopp\nESC: Avslutt\n")
    stdscr.refresh()

    try:
        while True:
            key = stdscr.getch()
            if key in (ord('w'), ord('W')):
                motor.send_command(SPEED, 0, 0)
                stdscr.addstr(8, 0, "Kjører fremover       ")
            elif key in (ord('s'), ord('S')):
                motor.send_command(-SPEED, 0, 0)
                stdscr.addstr(8, 0, "Kjører bakover        ")
            elif key in (ord('e'), ord('E')):
                motor.send_command(0, 0, ROTATION_SPEED)
                stdscr.addstr(8, 0, "Roterer venstre       ")
            elif key in (ord('q'), ord('Q')):
                motor.send_command(0, 0, -ROTATION_SPEED)
                stdscr.addstr(8, 0, "Roterer høyre         ")
            elif key in (ord('x'), ord('X')):
                motor.send_command(0, 0, 0)
                stdscr.addstr(8, 0, "Stopper               ")
            elif key == 27:  # ESC-tasten
                motor.send_command(0, 0, 0)
                stdscr.addstr(8, 0, "Avslutter manuell kontroll...   ")
                stdscr.refresh()
                break
            stdscr.refresh()
            time.sleep(0.1)
    finally:
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()
        curses.endwin()

# ----------------- HOVEDPROGRAM -----------------
def main():
    try:
        motor = MotorController()
        logging.info("Starter manuell kontrollmodus. Trykk ESC for å avslutte.")
        keyboard_control_mode(motor)
    except Exception as e:
        logging.error(f"Feil: {e}")
    finally:
        if 'motor' in locals():
            motor.close()

if __name__ == "__main__":
    main()
