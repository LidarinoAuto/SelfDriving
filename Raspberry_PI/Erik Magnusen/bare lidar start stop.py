import RPi.GPIO as GPIO
import serial
import time
import threading
from rplidar import RPLidar

# ----------------- LIDAR-konfigurasjon -----------------
PORT_NAME = "/dev/ttyUSB1"  # Riktig port for LIDAR
lidar = RPLidar(PORT_NAME, baudrate=115200)

# ----------------- Serial til ESP32 -----------------
ESP_PORT = "/dev/ttyUSB0"  # Riktig port for ESP32
esp = serial.Serial(ESP_PORT, 115200, timeout=1)
time.sleep(2)  # La ESP32 starte opp

# ----------------- Parametere -----------------
STOP_DISTANCE_LIDAR = 250       # Lidar: 250 mm
# Merk: �vrige parametere for motor/ultralyd fjernes eller ignoreres i denne modusen.
SPEED = 100
FORWARD_DISTANCE_AFTER_TURN = 10
TIME_TO_MOVE_1M = FORWARD_DISTANCE_AFTER_TURN / SPEED
ROTATION_SPEED = 2
ROTATION_TIME_90_DEG = 1.5

current_distance_lidar = 9999
running = False    # Styrer LIDAR-tr�den
lidar_mode = False # Globalt flagg for om LIDAR-modus (bare LIDAR) er aktiv

# ----------------- Ultralydsensor-konfigurasjon -----------------
# (Disse funksjonene forblir her, men de brukes ikke i "bare LIDAR"-modus)
trig_pins = [9, 7, 23, 10]
echo_pins = [8, 6, 24, 11]

def setup_ultrasound():
    GPIO.setmode(GPIO.BCM)
    for trig in trig_pins:
        GPIO.setup(trig, GPIO.OUT)
        GPIO.output(trig, False)
    for echo in echo_pins:
        GPIO.setup(echo, GPIO.IN)
    time.sleep(2)

def les_avstand(trig, echo):
    GPIO.output(trig, False)
    time.sleep(0.000002)
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    
    timeout_start = time.time()
    while GPIO.input(echo) == 0:
        pulse_start = time.time()
        if pulse_start - timeout_start > 0.03:
            return -1

    timeout_start = time.time()
    while GPIO.input(echo) == 1:
        pulse_end = time.time()
        if pulse_end - timeout_start > 0.03:
            return -1

    duration = (pulse_end - pulse_start) * 1e6  # konverter til mikrosekunder
    if duration >= 38000:
        return -1
    else:
        return duration / 58.0  # Avstand i cm

# ----------------- LIDAR-tr�d -----------------
def lidar_thread():
    global current_distance_lidar, running
    # LIDAR-skanning kj�rer s� lenge 'running' er True.
    for scan in lidar.iter_scans():
        if not running:
            break
        distances = []
        for measurement in scan:
            angle = measurement[1]  # vinkel (grader)
            distance = measurement[2]  # avstand (mm)
            # Vurder kun m�linger rett frem (�30�)
            if abs(angle - 0) <= 30 or abs(angle - 360) <= 30:
                distances.append(distance)
        if distances:
            current_distance_lidar = min(distances)

# ----------------- Seriell lytter for LIDAR-start/stopp -----------------
def serial_listener():
    global lidar_mode, running, lidar_thread_obj
    while True:
        if esp.in_waiting:
            try:
                # Les �n linje fra ESP og fjern linjeskift
                command = esp.readline().strip().decode('utf-8')
            except Exception as e:
                print("Feil ved lesing fra ESP:", e)
                command = ""
            if command == "START_LIDAR":
                if not lidar_mode:
                    print("Mottok 'START_LIDAR'. Starter LIDAR.")
                    lidar_mode = True
                    running = True
                    lidar_thread_obj = threading.Thread(target=lidar_thread)
                    lidar_thread_obj.daemon = True
                    lidar_thread_obj.start()
            elif command == "STOP_LIDAR":
                if lidar_mode:
                    print("Mottok 'STOP_LIDAR'. Stopper LIDAR.")
                    lidar_mode = False
                    running = False
                    try:
                        lidar.stop()
                        lidar.stop_motor()
                    except Exception as e:
                        print("Feil ved stopp av LIDAR:", e)
        time.sleep(0.1)

# ----------------- HOVEDPROGRAM -----------------
try:
    setup_ultrasound()
    # Start seriell lytter i egen tr�d for � fange opp kommandoer fra ESP32
    serial_thread = threading.Thread(target=serial_listener)
    serial_thread.daemon = True
    serial_thread.start()
    
    # Hovedl�kken utf�rer n� kun LIDAR-relatert oppdatering og presentasjon.
    while True:
        if lidar_mode:
            # Bare vis LIDAR-avstanden; ingen motorstyring eller ultralydkontroll.
            print(f"LIDAR-avstand: {current_distance_lidar:.1f} mm")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Avslutter programmet...")
    
finally:
    running = False
    try:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
    except Exception as e:
        print("Feil ved stopp/avkobling av LIDAR:", e)
    esp.close()
    GPIO.cleanup()
