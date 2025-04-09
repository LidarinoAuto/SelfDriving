import RPi.GPIO as GPIO
import serial
import time
import threading
import random
from rplidar import RPLidar
import smbus
import math

# ----------------- LIDAR-konfigurasjon -----------------
PORT_NAME = "/dev/ttyUSB1"  # Riktig port for LIDAR
lidar = RPLidar(PORT_NAME, baudrate=115200)

# Global variabel for siste LIDAR-skanning
latest_scan = []

# ----------------- Serial til ESP32 -----------------
ESP_PORT = "/dev/ttyUSB0"  # Riktig port for ESP32
esp = serial.Serial(ESP_PORT, 115200, timeout=1)
time.sleep(2)  # La ESP32 starte opp

# ----------------- Parametere for hindringsdeteksjon -----------------
STOP_DISTANCE_LIDAR = 250              # Lidar: 250 mm (25 cm)
STOP_DISTANCE_ULTRASOUND = 15          # Ultralyd: 15 cm
SPEED = 100                            # mm/s, fremoverhastighet
FORWARD_DISTANCE_AFTER_TURN = 10       # mm, kort strekning etter rotasjon
TIME_TO_MOVE_1M = FORWARD_DISTANCE_AFTER_TURN / SPEED  # sekunder
ROTATION_SPEED = 2                     # Rotasjonshastighet (brukes i kommandoer til ESP32)
ROTATION_TIME_90_DEG = 1.5             # Tid for en 90� rotasjon

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

def check_front_ultrasound():
    """
    Returnerer True dersom frontsensorene (sensor 0 og 1) registrerer hindring.
    """
    for i in [0, 1]:
        d = les_avstand(trig_pins[i], echo_pins[i])
        if 2 < d < STOP_DISTANCE_ULTRASOUND:
            print(f"[ULTRA] Hindring oppdaget av sensor {i}: {d:.2f} cm")
            return True
    return False

def get_best_ultrasound_direction():
    """
    Leser alle ultralydsensorene og returnerer et tuple:
    (best_index, best_distance) der 'best_index' er indeksen til den sensoren
    med den st�rste (mest frie) avstanden og 'best_distance' er avstanden.
    Dersom frontsensorene (sensor 0 og 1) er blokkert, benyttes bakre sensorer.
    Inneholder ogs� et enkelt st�yfilter.
    """
    sensor_names = ["front_left", "front_right", "back_left", "back_right"]
    distances = []
    
    for i in range(len(trig_pins)):
        d = les_avstand(trig_pins[i], echo_pins[i])
        # St�yfilter: ugyldige m�linger
        if d < 2 or d > 300:
            d = -1
        distances.append(d)
        if d != -1:
            print(f"{sensor_names[i]}: {d:.2f} cm")
        else:
            print(f"{sensor_names[i]}: ugyldig m�ling")
    
    # Prioriter frontsensorene
    if distances[0] > STOP_DISTANCE_ULTRASOUND or distances[1] > STOP_DISTANCE_ULTRASOUND:
        best_index = 0 if distances[0] > distances[1] else 1
        return best_index, distances[best_index]
    
    # Om frontsensorene er blokkert, benytt bakre sensorer
    if distances[2] > distances[3]:
        return 2, distances[2]
    else:
        return 3, distances[3]

def check_ultrasound():
    """
    Ekstra sikkerhetssjekk med alle sensorene.
    """
    for i in range(len(trig_pins)):
        d = les_avstand(trig_pins[i], echo_pins[i])
        if 2 < d < STOP_DISTANCE_ULTRASOUND:
            print(f"Ultralydsensor {i} oppdager hindring: {d:.2f} cm")
            return True
    return False

# ----------------- LIDAR-tr�d -----------------
def lidar_thread():
    global current_distance_lidar, running, latest_scan
    for scan in lidar.iter_scans():
        latest_scan = scan  # lagrer hele skanningen for videre analyse
        distances = []
        for measurement in scan:
            angle = measurement[1]  # vinkel (grader)
            distance = measurement[2]  # avstand (mm)
            # Vurder kun m�linger rett fram (�30�)
            if abs(angle - 0) <= 30 or abs(angle - 360) <= 30:
                distances.append(distance)
        if distances:
            current_distance_lidar = min(distances)
        if not running:
            break

def get_best_lidar_direction():
    """
    Analyserer siste LIDAR-skanning og finner vinkel med maksimalt fri bane.
    Returnerer et tuple (vinkel, avstand).
    Vinkelen regnes ut i forhold til front (0�); dersom verdien er over 180,
    trekkes 360 fra for � f� et negativt utslag (venstresving).
    """
    global latest_scan
    if not latest_scan:
        return 0, 9999
    
    best_angle = 0
    best_distance = -1
    for measurement in latest_scan:
        angle = measurement[1]
        distance = measurement[2]
        if distance > best_distance:
            best_distance = distance
            best_angle = angle
    # Juster vinkel slik at verdier over 180 (h�yre) gis et negativt utslag
    if best_angle > 180:
        best_angle -= 360
    return best_angle, best_distance

# ----------------- Motorstyring (via ESP32) -----------------
def send_command(command):
    print(f"Sender til ESP32: {command.strip()}")
    esp.write(command.encode())

# ----------------- IMU-konfigurasjon (MPU6050 p� adresse 0x68) -----------------
IMU_ADDRESS = 0x68
bus = smbus.SMBus(1)

def init_imu():
    """
    Initialiserer MPU6050 ved � skrive til registeret 0x6B for � v�kne opp sensoren.
    """
    bus.write_byte_data(IMU_ADDRESS, 0x6B, 0)
    time.sleep(0.1)

def read_word(adr):
    high = bus.read_byte_data(IMU_ADDRESS, adr)
    low = bus.read_byte_data(IMU_ADDRESS, adr + 1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val

def get_gyro_z():
    """
    Leser gyroskopdata for z-aksen (yaw). Konverterer til grader per sekund.
    """
    gyro_z = read_word_2c(0x47)
    return gyro_z / 131.0

def rotate_by_angle(angle):
    """
    Roterer roboten med angitt vinkel (grader) ved � bruke gyroskopdata.
    Negativ verdi = venstresving, positiv verdi = h�yresving.
    Rotasjonstiden beregnes proporsjonalt med 90�.
    """
    rotation_time = (abs(angle) / 90) * ROTATION_TIME_90_DEG
    if angle < 0:
        send_command(f"0 0 {ROTATION_SPEED}\n")
    else:
        send_command(f"0 0 {-ROTATION_SPEED}\n")

    start_time = time.time()
    last_time = start_time
    integrated_angle = 0.0

    while time.time() < start_time + rotation_time:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        angular_rate = get_gyro_z()  # grader per sekund
        integrated_angle += angular_rate * dt
        time.sleep(0.01)

    send_command("0 0 0\n")
    print(f"Integrert rotasjon: {integrated_angle:.2f} grader")

def choose_direction_and_rotate(sensor):
    """
    Velger ny retning basert p� sensoren som oppdaget hindringen.
    Hvis 'sensor' er 'ultrasound', benyttes ultralyddata.
    Hvis 'sensor' er 'lidar', benyttes LIDAR-scan for � finne retning.
    """
    if sensor == 'ultrasound':
        best_index, best_distance = get_best_ultrasound_direction()
        # Kartlegging fra sensor til rotasjonsvinkel:
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
        print(f"[ULTRA] Beste retning: sensor {best_index} med {best_distance:.2f} cm, roterer {angle}�")
    elif sensor == 'lidar':
        angle, best_distance = get_best_lidar_direction()
        print(f"[LIDAR] Beste retning: vinkel {angle:.2f}�, avstand {best_distance:.2f} mm")
    else:
        angle = 0
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
    init_imu()  # Initialiser IMU for rotasjonsm�ling
    lidar_thread_obj = threading.Thread(target=lidar_thread)
    lidar_thread_obj.daemon = True
    lidar_thread_obj.start()
    
    # Start roboten i fremoverbevegelse
    move_forward()
    
    while True:
        # Sjekk om hindring oppdages av ultralyd eller LIDAR
        ultra_trigger = check_front_ultrasound()
        lidar_trigger = (current_distance_lidar <= STOP_DISTANCE_LIDAR)
        
        if ultra_trigger or lidar_trigger:
            stop_robot()
            time.sleep(1)
            # Prioriter ultralyd dersom begge trigges
            if ultra_trigger:
                choose_direction_and_rotate('ultrasound')
            else:
                choose_direction_and_rotate('lidar')
            move_forward_distance()
            move_forward()
            continue
        
        print(f"LIDAR-avstand: {current_distance_lidar:.1f} mm")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Avslutter programmet...")
    stop_robot()
    
     (fimport RPi.GPIO as GPIO
import serial
import time
import threading
import random
from rplidar imp