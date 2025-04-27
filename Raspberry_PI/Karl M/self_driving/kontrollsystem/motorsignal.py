# motorsignal.py
import serial

PORT = "/dev/esp32"  # Juster dette til riktig port!
BAUDRATE = 115200

def send_movement_command(x, y, omega):
    message = f"{x} {y} {omega:.2f}\n".encode()
    try:
        with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
            ser.write(message)
            print(f"Sendt: {x} {y} {omega:.2f}")
    except Exception as e:
        print(f"Feil ved sending til ESP32: {e}")
