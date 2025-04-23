import serial
import time

# Konfigurasjon
ESP_PORT = "/dev/esp32"      # Juster til riktig seriell-port
BAUDRATE = 115200
SPEED = 100                  # Hastighet fremover/bakover (mm/s)
ROT_SPEED = 45.5             # Rotasjonshastighet (grader/s), kan v�re float

def send_command(esp, x, y, w):
    """
    Sender en kommando til ESP32:
      x = frem/bak (+ = fremover, - = bakover)
      y = sidelengs (aldri brukt her, settes til 0)
      w = rotasjon (+ = venstre, - = h�yre) i grader/s
    """
    cmd = f"{x} {y} {w:.2f}\n".encode()
    esp.write(cmd)
    print(f"Sendt: {x} {y} {w:.2f}")

def stop(esp):
    """Stopper all bevegelse."""
    send_command(esp, 0, 0, 0.0)

def main():
    # �pne seriell-tilkobling
    with serial.Serial(ESP_PORT, BAUDRATE, timeout=1) as esp:
        time.sleep(2)  # Gi ESP32 tid til � starte opp
        
        # Fremover i 2 sek
        send_command(esp,  SPEED, 0, 0.0)
        time.sleep(2)
        stop(esp)
        time.sleep(0.5)
        
        # Bakover i 2 sek
        send_command(esp, -SPEED, 0, 0.0)
        time.sleep(2)
        stop(esp)
        time.sleep(0.5)
        
        # Sidelengs h�yre i 2 sek (positiv y)
        send_command(esp, 0, SPEED, 0.0)
        time.sleep(2)
        stop(esp)
        time.sleep(0.5)

        # Sidelengs venstre i 2 sek (negativ y)
        send_command(esp, 0, -SPEED, 0.0)
        time.sleep(2)
        stop(esp)
        
        # H�yre (rotasjon) i 2 sek
        # Negativ w = h�yresving
        send_command(esp, 0, 0, -ROT_SPEED)
        time.sleep(2)
        stop(esp)
        time.sleep(0.5)
        
        # Venstre (rotasjon) i 2 sek
        # Positiv w = venstresving
        send_command(esp, 0, 0, ROT_SPEED)
        time.sleep(2)
        stop(esp)

if __name__ == "__main__":
    main()
