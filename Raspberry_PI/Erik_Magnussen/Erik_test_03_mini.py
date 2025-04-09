import RPi.GPIO as GPIO
import serial
import time
import threading
import random
import math
import smbus
from rplidar import RPLidar

# ----------------- LIDAR-konfigurasjon -----------------
PORT_NAME = "/dev/ttyUSB0"  # Riktig port for LIDAR
lidar = RPLidar(PORT_NAME, baudrate=115200)

# ----------------- Serial til ESP32 -----------------
ESP_PORT = "/dev/ttyUSB1"  # Riktig port for ESP32
esp = serial.Serial(ESP_PORT, 115200, timeout=1)
time.sleep(2)  # La ESP32 starte opp

# ----------------- Parametere for hindringsdeteksjon -----------------
STOP_DISTANCE_LIDAR = 250       # Lidar: 250 mm
STOP_DISTANCE_ULTRASOUND = 15     # Ultralyd: 15 cm
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
    Velger den retningen med minst hindring basert p� ultralydsensorene og roterer roboten i den retningen.
    """
    best_index, best_distance = get_best_ultrasound_direction()
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
    
import smbus
import time
import math

# ----------------- Kompass (QMC5883L) -----------------
QMC5883L_ADDRESS = 0x0D  # Standard I2C-adresse for QMC5883L
bus = smbus.SMBus(1)    # Bruk I2C-buss 1

# Registeradresser for QMC5883L
QMC5883L_CTRL1 = 0x09
QMC5883L_SET_RESET = 0x0B
QMC5883L_DATA = 0x00   # Data starter her (6 bytes)

def setup_hmc5883l():
    """Initialiserer QMC5883L med riktige innstillinger."""
    # Soft reset
    bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_SET_RESET, 0x01)
    # 0b00011101: 10Hz oppdatering, 128x oversampling, kontinuerlig m�ling
    bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_CTRL1, 0b00011101)
    time.sleep(0.5)

def read_compass():
    """Leser 6 bytes fra QMC5883L og beregner heading i grader."""
    data = bus.read_i2c_block_data(QMC5883L_ADDRESS, QMC5883L_DATA, 6)
    # QMC5883L sender data i LITTLE-ENDIAN rekkef�lge
    x = (data[1] << 8) | data[0]
    y = (data[3] << 8) | data[2]
    z = (data[5] << 8) | data[4]
    
    # H�ndterer to-komplement for negative verdier
    if x > 32767: 
        x -= 65536
    if y > 32767: 
        y -= 65536
    if z > 32767: 
        z -= 65536
    
    heading = math.atan2(y, x) * (180 / math.pi)
    if heading < 0:
        heading += 360
    heading =(heading-218) %300   
    return heading

def get_cardinal_direction(heading):
    """Konverterer grader til en kardinalretning (Nord, �st, S�r, Vest)."""
    if heading >= 315 or heading < 45:
        return "Nord"
    elif heading >= 45 and heading < 135:
        return "�st"
    elif heading >= 135 and heading < 225:
        return "S�r"
    else:
        return "Vest"


# ----------------- HOVEDPROGRAM -----------------
try:
    setup_ultrasound()
    setup_hmc5883l()  # Initialiserer kompasset
    lidar_thread_obj = threading.Thread(target=lidar_thread)
    lidar_thread_obj.daemon = True
    lidar_thread_obj.start()
    
    # Starter med � kj�re roboten fremover
    move_forward()
    
    while True:
        # Les og presenter kompassretning
        compass_heading = read_compass()
        cardinal = get_cardinal_direction(compass_heading)
        print(f"Kompassretning: {compass_heading:.2f}� ({cardinal})")
        
        # Prioriterer ultralyd: Hvis en hindring oppdages, stopp og korriger retning
        if check_ultrasound():
            stop_robot()
            time.sleep(2)  # �kt ventetid for stabilisering av sensoravlesningene
            choose_direction_and_rotate()
            move_forward_distance()
            move_forward()
            continue
        
        # Sjekker LIDAR dersom ingen hindring oppdages med ultralyd
        print(f"LIDAR-avstand: {current_distance_lidar:.1f} mm")
        if current_distance_lidar <= STOP_DISTANCE_LIDAR:
            stop_robot()
            time.sleep(2)  # �kt ventetid f�r korrigerende man�ver
            choose_direction_and_rotate()
            move_forward_distance()
            move_forward()
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Avslutter programmet...")
    stop_robot()
    
finally:
    running = False
    lidar.stop()          # Stopper LIDAR-skanningen
    lidar.stop_motor()    # Stopper LIDAR-motoren
    lidar.disconnect()    # Koble fra LIDAR
    esp.close()           # Lukker seriekoblingen til ESP32
    GPIO.cleanup()        # Rydder opp i GPIO-innstillingene
