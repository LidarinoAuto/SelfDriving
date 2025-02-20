import serial
import time
import threading
from rplidar import RPLidar

# Sett opp LIDAR
PORT_NAME = "/dev/ttyUSB0"  # Sjekk at dette er riktig port!
lidar = RPLidar(PORT_NAME, baudrate=115200)

# Sett opp Serial til ESP32
ESP_PORT = "/dev/ttyUSB1"  # Sjekk at dette er riktig port!
esp = serial.Serial(ESP_PORT, 115200, timeout=1)
time.sleep(2)  # Vent for å sikre at Serial er klar

# Minimum avstand til hindring (30 cm = 300 mm)
STOP_DISTANCE = 300

# Definer hvilken vinkel vi skal sjekke for hindringer (forover = 0° ± 20°)
FORWARD_ANGLE = 0  # Y-retning
ANGLE_TOLERANCE = 20  # Hvor mye til hver side vi godtar

# Variabel for å holde styr på nærmeste avstand
current_distance = 9999
running = True  # Kontroll for tråden


def lidar_thread():
    """LIDAR kjører i en egen tråd for å kontinuerlig oppdatere avstanden."""
    global current_distance, running

    for scan in lidar.iter_scans():
        distances = []

        for measurement in scan:
            angle = measurement[1]  # Vinkel på målingen (grader)
            distance = measurement[2]  # Avstand (mm)

            # Sjekk om vinkelen er innenfor "forover"-området
            if abs(angle - FORWARD_ANGLE) <= ANGLE_TOLERANCE or abs(angle - 360 + FORWARD_ANGLE) <= ANGLE_TOLERANCE:
                distances.append(distance)

        if distances:
            current_distance = min(distances)  # Oppdater global variabel

        if not running:
            break  # Avslutt tråden hvis programmet stopper


def send_command(command):
    """Sender kommando til ESP32 via Serial og logger det."""
    print(f"🛠 Sender til ESP32: {command.strip()}")
    esp.write(command.encode())


try:
    # Start LIDAR-tråden for kontinuerlig avlesning
    print("📡 Starter LIDAR-tråd for kontinuerlig avlesning...")
    thread = threading.Thread(target=lidar_thread)
    thread.daemon = True
    thread.start()

    # Sikre at roboten starter fra stoppet
    print("🔎 Tester retning... Stopper først.")
    send_command("0 0 0\n")
    time.sleep(1)

    print("🚀 Starter roboten... Kjører forover (Y = 100)")
    send_command("0 100 0\n")

    while True:
        print(f"📏 Avstand til hindring i Y-retning: {current_distance:.1f} mm")

        if current_distance <= STOP_DISTANCE:  # Hvis avstand er under 30 cm
            print("🚨 Hindring oppdaget i kjøreretning! Stopper roboten.")
            send_command("0 0 0\n")  # Send stopp-kommando
            break

        time.sleep(0.1)  # Kort pause

except KeyboardInterrupt:
    print("🛑 Avslutter programmet...")
    send_command("0 0 0\n")  # Send stopp-signal ved avbrudd

finally:
    running = False  # Stopp LIDAR-tråden
    lidar.stop()
    lidar.disconnect()
    esp.close()
