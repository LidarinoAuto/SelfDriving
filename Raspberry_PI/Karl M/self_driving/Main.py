# Main.py
from motorsignal import send_movement_command

def main():
    # Eksempel på en enkel bevegelseskommando
    x_speed = 100      # mm/s
    y_speed = 0        # mm/s
    omega = 45         # grader/s

    send_movement_command(x_speed, y_speed, omega)
    print("Kommando sendt til ESP32")

if __name__ == "__main__":
    main()
