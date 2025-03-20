import RPi.GPIO as GPIO
import serial
import time
import threading
import random
from rplidar import RPLidar

# ----------------- LIDAR-konfigurasjon -----------------
PORT_NAME = "/dev/ttyUSB1"  # Riktig port for LIDAR
lidar = RPLidar(PORT_NAME, baudrate=115200)

# ----------------- Serial til ESP32 -----------------
ESP_PORT = "/dev/ttyUSB0"  # Riktig port for ESP32
esp = serial.Serial(ESP_PORT, 115200, timeout=1)
time.sleep(2)  # La ESP32 starte opp

# ----------------- Parametere for hindringsdeteksjon -----------------
STOP_DISTANCE_LIDAR =    250 # Lidar: 250 mmm (25 cm)
STOP_DISTANCE_ULTRASOUND = 100    # Ultralyd: 100 cm (10 cm)
SPEED = 100                       # mm/s, fremoverhastighet
FORWARD_DISTANCE_AFTER_TURN = 10  # mm, kort strekning etter rotasjon
TIME_TO_MOVE_1M = FORWARD_DISTANCE_AFTER_TURN / SPEED  # sekunder
ROTATION_SPEED = 2                # Rotasjonshastighet (brukes i kommandoer til ESP32)
ROTATION_TIME_90_DEG = 1.5        # Tid for en 90� rotasjon

current_distance_lidar = 9999
running = True

# ----------------- Ultralydsensor-konfigurasjon -----------------
# Sensorrekkef�lge: front venstre, front h�yre, bak venstre, bak h�yre
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

def get_best_ultrasound_direction():
    """
    Leser alle ultralydsensorene og returnerer et tuple:
    (best_index, best_distance) der 'best_index' er indeksen til den sensoren
    som m�ler den st�rste (dvs. mest fri) avstanden, og 'best_distance' er avstanden.
    """
    sensor_names = ["front_left", "front_right", "back_left", "back_right"]
    distances = []
    for i in range(len(trig_pins)):
        d = les_avstand(trig_pins[i], echo_pins[i])
        distances.append(d)
        print(f"{sensor_names[i]}: {d:.2f} cm")
    
    best_index = 0
    best_distance = distances[0] if distances[0] > 0 else 0
    for i, d in enumerate(distances):
        if d > best_distance:
            best_distance = d
            best_index = i

    return best_index, best_distance

def check_ultrasound():
    """
    Returnerer True dersom noen av ultralydsensorene registrerer en hindring
    innenfor STOP_DISTANCE_ULTRASOUND.
    """
    for i in range(len(trig_pins)):
        d = les_avstand(trig_pins[i], echo_pins[i])
        if d > 0 and d < STOP_DISTANCE_ULTRASOUND:
            print(f"Ultralydsensor {i} oppdager hindring: {d:.2f} cm")
            return True
    return False

# ----------------- LIDAR-tr�d -----------------
def lidar_thread():
    global current_distance_lidar, running
    for scan in lidar.iter_scans():
        distances = []
        for measurement in scan:
            angle = measurement[1]  # vinkel (grader)
            distance = measurement[2]  # avstand (mm)
            # Vurder kun m�linger rett frem (�30�)
            if abs(angle - 0) <= 30 or abs(angle - 360) <= 30:
                distances.append(distance)
        if distances:
            current_distance_lidar = min(distances)
        if not running:
            break

# ----------------- Motorstyring (via ESP32) -----------------
def send_command(command):
    print(f"Sender til ESP32: {command.strip()}")
    esp.write(command.encode())

def rotate_by_angle(angle):
    """
    Roterer roboten med angitt vinkel i grader.
    Negativ verdi = venstresving, positiv = h�yresving.
    Rotasjonstiden beregnes proporsjonalt med 90�.
    """
    rotation_time = (abs(angle) / 90) * ROTATION_TIME_90_DEG
    if angle < 0:
        send_command(f"0 0 {ROTATION_SPEED}\n")
    else:
        send_command(f"0 0 {-ROTATION_SPEED}\n")
    time.sleep(rotation_time)
    send_command("0 0 0\n")

def choose_direction_and_rotate():
    """
    Velger den retningen med minst hindring basert p� ultralydsensorene og roterer
    roboten i den retningen.
    """
    best_index, best_distance = get_best_ultrasound_direction()
    # Kartlegging av sensorindeks til �nsket rotasjonsvinkel:
    # front_left -> sving ca. 30� venstre, front_right -> sving ca. 30� h�yre
    # back_left -> sving ca. 150� venstre, back_right -> sving ca. 150� h�yre
    if best_index == 0:
        angle = -30
    elif best_index == 1:
        angle = 30
    elif best_index == 2:
        angle = -150
    elif best_index == 3:
        angle = 150
    else:
        angle = 0
    print(f"Beste retning: sensor {best_index} med {best_distance:.2f} cm, roterer {angle}�")
    rotate_by_angle(angle)

def move_forward():
    print("Kj�rer fremover")
    send_command("100 0 0\n")

def stop_robot():
    print("Stopper roboten")
    send_command("0 0 0\n")

def move_forward_distance():
    print("Kj�rer et kort stykke fremover etter rotasjon")
    send_command(f"{SPEED} 0 0\n")
    time.sleep(TIME_TO_MOVE_1M)
    send_command("0 0 0\n")

# ----------------- HOVEDPROGRAM -----------------
try:
    setup_ultrasound()
    lidar_thread_obj = threading.Thread(target=lidar_thread)
    lidar_thread_obj.daemon = True
    lidar_thread_obj.start()
    
    # Start med � kj�re fremover
    move_forward()
    
    while True:
        # Prioriter ultralyd: Hvis en hindring oppdages, stopp og finn fri retning
        if check_ultrasound():
            stop_robot()
            time.sleep(1)
            choose_direction_and_rotate()
            move_forward_distance()
            move_forward()
            continue
        
        # Sjekk LIDAR hvis ingen hindring oppdages med ultralyd
        print(f"LIDAR-avstand: {current_distance_lidar:.1f} mm")
        if current_distance_lidar <= STOP_DISTANCE_LIDAR:
            stop_robot()
            time.sleep(1)
            choose_direction_and_rotate()
            move_forward_distance()
            move_forward()
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Avslutter programmet...")
        
    # Legg inn en forsinkelse p� 5 sekunder f�r LIDAR-rotasjonen stoppes
    print("Venter 5 sekunder f�r LIDAR-rotasjonen stopper...")
    time.sleep(5)
    
    #lidar.disconnect()
    lidar.stop()
    lidar.stop_motor()
    stop_robot()

finally:
    running = False  # Stopp LIDAR-tr�den f�rst
    lidar_thread_obj.join()  # Vent til tr�den avslutter
    lidar.stop()
    lidar.disconnect()
    #esp.close()
   # GPIO.cleanup()

