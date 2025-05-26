# Denne modulen h�ndterer grunnleggende motorstyring via seriell kommunikasjon med en ESP32
import serial
import time
from logging_utils import skriv_logg

# Sett opp seriekoblingen mot ESP32
ESP_PORT = "/dev/esp32"  # Tilpass portnavn etter systemet ditt (f.eks. "COM3" p� Windows)
esp = serial.Serial(ESP_PORT, 115200, timeout=1)
time.sleep(2)  # Vent kort for � sikre at forbindelsen er klar

# Sender en kommando til ESP32 over seriell
def send_command(command):
    #skriv_logg(f"Sender: {command.strip()}")
    try:
        esp.write(command.encode())
    except serial.SerialException as e:
        skriv_logg(f"SERIELL FEIL: Kunne ikke sende kommando '{command.strip()}': {e}")
    except Exception as e:
        skriv_logg(f"Ukjent feil ved sending av kommando '{command.strip()}': {e}")

# Beveg roboten fremover
def move_forward():
    send_command("100 0 0\n")  # Eksempel: Venstre = 100, H�yre = 100, Rotasjon = 0

# Stopp all bevegelse
def stop_robot():
    send_command("0 0 0\n")