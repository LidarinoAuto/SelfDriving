# motorsignal.py
import serial

# Oppdater med riktig port, f.eks. /dev/ttyUSB0 eller /dev/ttyACM0
PORT = "/dev/esp32"
BAUDRATE = 115200

def send_movement_command(x, y, omega):
    message = f"{x},{y},{omega}\n"
    try:
        with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
            ser.write(message.encode())
    except Exception as e:
        print(f"Feil ved sending til ESP32: {e}")
