import RPi.GPIO as GPIO
import serial
import time
import threading
import math
import smbus
import numpy as np
import logging
from rplidar import RPLidar
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

# Konfigurer logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# ----------------- InfluxDB-innstillinger -----------------
INFLUX_URL = "https://eu-central-1-1.aws.cloud2.influxdata.com/"
INFLUX_TOKEN = "ZbCkxFHS5X0_9PG3YMeznY07WFCsMfaV7MMTSWyb7Ckq72zxMdVq2rB20ZdyxccfB1di9wQOBguZv70A9VEErA=="
INFLUX_ORG = "61778c39081df8c1"
INFLUX_BUCKET = "Lidarino"

client = InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG)
write_api = client.write_api(write_options=SYNCHRONOUS)

# ----------------- LIDAR-konfigurasjon -----------------
PORT_NAME = "/dev/ttyUSB1"  # Riktig port for LIDAR
lidar = RPLidar(PORT_NAME, baudrate=115200)

# ----------------- Serial til ESP32 -----------------
ESP_PORT = "/dev/ttyUSB0"  # Riktig port for ESP32
esp = serial.Serial(ESP_PORT, 115200, timeout=1)
time.sleep(2)  # La ESP32 starte opp

# ----------------- Parametere for hindringsdeteksjon -----------------
STOP_DISTANCE_LIDAR = 250       # mm
STOP_DISTANCE_ULTRASOUND = 15     # cm
SPEED = 100                       # mm/s (fremover)
FORWARD_DISTANCE_AFTER_TURN = 10  # mm
TIME_TO_MOVE_SHORT = FORWARD_DISTANCE_AFTER_TURN / SPEED  # sekunder
ROTATION_SPEED = 2                # (til ESP32-kommandoer)
ROTATION_TIME_90_DEG = 1.5        # Tid for en 90�-rotasjon

current_distance_lidar = 9999
running = True

# Global pose representert som en 3x3 SE2-matrise (identitetsmatrise ved start)
current_pose = np.eye(3)

# ----------------- Definer sensorpinner (ultralyd) -----------------
SENSORS = {
    "FrontVenstre": {"trig": 9, "echo": 8},
    "FrontHoyre": {"trig": 7, "echo": 6},
    "BakVenstre": {"trig": 23, "echo": 24},
    "BakHoyre": {"trig": 10, "echo": 11}
}

# ----------------- Funksjoner for SE2-transformasjon og twist-integrasjon -----------------
def se2_transform(x, y, theta):
    """Returnerer en 3x3 homogen transformasjonsmatrise for posisjon (x, y) og orientering theta (i rad)."""
    return np.array([[math.cos(theta), -math.sin(theta), x],
                     [math.sin(theta),  math.cos(theta), y],
                     [0, 0, 1]])

def twist_to_transform(vx, vy, omega, dt):
    """
    Konverterer en twist (vx, vy, omega) og tidssteg dt til en SE2-transformasjon.
    vx og vy i mm/s, omega i rad/s.
    """
    if abs(omega) < 1e-6:
        dx = vx * dt
        dy = vy * dt
        dtheta = 0
    else:
        dtheta = omega * dt
        dx = (vx * math.sin(dtheta) + vy * (1 - math.cos(dtheta))) / omega
        dy = (vy * math.sin(dtheta) - vx * (1 - math.cos(dtheta))) / omega
    return se2_transform(dx, dy, dtheta)

def update_pose(vx, vy, omega, dt):
    """Oppdaterer den globale pose basert p� den m�lte twist og tidssteg dt."""
    global current_pose
    increment = twist_to_transform(vx, vy, omega, dt)
    current_pose = current_pose @ increment
    x, y, theta = get_pose_parameters(current_pose)
    logging.info(f"Oppdatert pose: x={x:.1f} mm, y={y:.1f} mm, theta={math.degrees(theta):.1f}�")

def get_pose_parameters(pose):
    """Ekstraherer x, y og theta fra en SE2-matrise."""
    x = pose[0, 2]
    y = pose[1, 2]
    theta = math.atan2(pose[1, 0], pose[0, 0])
    return x, y, theta

# ----------------- Ultralydsensor-funksjoner -----------------
def setup_ultrasound():
    GPIO.setmode(GPIO.BCM)
    for sensor in SENSORS.values():
        GPIO.setup(sensor["trig"], GPIO.OUT)
        GPIO.output(sensor["trig"], False)
        GPIO.setup(sensor["echo"], GPIO.IN)
    time.sleep(2)

def les_avstand(trig, echo, retries=3):
    distances = []
    for _ in range(retries):
        GPIO.output(trig, False)
        time.sleep(0.000002)
        GPIO.output(trig, True)
        time.sleep(0.00001)
        GPIO.output(trig, False)
        
        timeout_start = time.time()
        while GPIO.input(echo) == 0:
            pulse_start = time.time()
            if pulse_start - timeout_start > 0.03:
                break

        timeout_start = time.time()
        while GPIO.input(echo) == 1:
            pulse_end = time.time()
            if pulse_end - timeout_start > 0.03:
                break

        duration = (pulse_end - pulse_start) * 1e6  # mikrosekunder
        if 0 < duration < 38000:
            distances.append(duration / 58.0)  # cm
        else:
            distances.append(-1)
        time.sleep(0.01)
    valid = [d for d in distances if d > 0]
    return sum(valid) / len(valid) if valid else -1

def read_all_ultrasound():
    """Leser alle ultralydsensorene og returnerer en dict med sensoravstander."""
    data = {}
    for navn, pinner in SENSORS.items():
        distance = les_avstand(pinner["trig"], pinner["echo"])
        data[navn] = distance if distance >= 0 else None
    return data

def get_best_ultrasound_direction():
    data = read_all_ultrasound()
    for navn, avstand in data.items():
        logging.info(f"{navn}: {avstand:.2f} cm" if avstand is not None else f"{navn}: Utenfor rekkevidde")
    best_sensor = max(data, key=lambda k: data[k] if data[k] is not None else -1)
    return best_sensor, data[best_sensor]

def check_ultrasound():
    """Returnerer True dersom minst �n ultralydsensor registrerer hindring innenfor STOP_DISTANCE_ULTRASOUND."""
    data = read_all_ultrasound()
    for navn, avstand in data.items():
        if avstand is not None and avstand < STOP_DISTANCE_ULTRASOUND:
            logging.info(f"{navn} oppdager hindring: {avstand:.2f} cm")
            return True
    return False

# ----------------- LIDAR-tr�d -----------------
def lidar_thread():
    global current_distance_lidar, running
    for scan in lidar.iter_scans():
        distances = [measurement[2] for measurement in scan
                     if abs(measurement[1] - 0) <= 30 or abs(measurement[1] - 360) <= 30]
        if distances:
            current_distance_lidar = min(distances)
        if not running:
            break

# ----------------- Motorstyring (via ESP32) -----------------
def send_command(command):
    logging.info(f"Sender til ESP32: {command.strip()}")
    esp.write(command.encode())

def rotate_by_angle(angle):
    """
    Roterer roboten med angitt vinkel (grader). 
    Beregner rotasjonstiden proporsjonalt med 90� og oppdaterer intern pose.
    """
    rotation_time = (abs(angle) / 90) * ROTATION_TIME_90_DEG
    omega = math.radians(angle) / rotation_time  # rad/s
    if angle < 0:
        send_command(f"0 0 {ROTATION_SPEED}\n")
    else:
        send_command(f"0 0 {-ROTATION_SPEED}\n")
    time.sleep(rotation_time)
    send_command("0 0 0\n")
    update_pose(0, 0, omega, rotation_time)

def choose_direction_and_rotate():
    best_sensor, best_distance = get_best_ultrasound_direction()
    if best_sensor == "FrontVenstre":
        angle = -30
    elif best_sensor == "FrontHoyre":
        angle = 30
    elif best_sensor == "BakVenstre":
        angle = -150
    elif best_sensor == "BakHoyre":
        angle = 150
    else:
        angle = 0
    logging.info(f"Beste retning: {best_sensor} med {best_distance:.2f} cm, roterer {angle}�")
    rotate_by_angle(angle)

def move_forward():
    logging.info("Kj�rer fremover")
    send_command("100 0 0\n")

def stop_robot():
    logging.info("Stopper roboten")
    send_command("0 0 0\n")

def move_forward_distance():
    logging.info("Kj�rer et kort stykke fremover etter rotasjon")
    send_command(f"{SPEED} 0 0\n")
    time.sleep(TIME_TO_MOVE_SHORT)
    send_command("0 0 0\n")
    update_pose(SPEED, 0, 0, TIME_TO_MOVE_SHORT)

# ----------------- Kompass (QMC5883L) -----------------
def setup_compass():
    QMC5883L_ADDRESS = 0x0D
    bus = smbus.SMBus(1)
    QMC5883L_CTRL1 = 0x09
    QMC5883L_SET_RESET = 0x0B
    bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_SET_RESET, 0x01)
    bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_CTRL1, 0b00011101)
    time.sleep(0.5)
    return bus

def read_compass(bus):
    QMC5883L_ADDRESS = 0x0D
    QMC5883L_DATA = 0x00
    data = bus.read_i2c_block_data(QMC5883L_ADDRESS, QMC5883L_DATA, 6)
    x = (data[1] << 8) | data[0]
    y = (data[3] << 8) | data[2]
    z = (data[5] << 8) | data[4]
    if x > 32767: x -= 65536
    if y > 32767: y -= 65536
    if z > 32767: z -= 65536
    heading = math.atan2(y, x) * (180 / math.pi)
    if heading < 0: heading += 360
    heading = (heading - 218) % 300
    return heading

def get_cardinal_direction(heading):
    if heading >= 315 or heading < 45:
        return "Nord"
    elif heading >= 45 and heading < 135:
        return "�st"
    elif heading >= 135 and heading < 225:
        return "S�r"
    else:
        return "Vest"

# ----------------- InfluxDB Logging -----------------
def log_debug_data(data):
    try:
        point = Point("debug_data")
        for key, value in data.items():
            if value is not None:
                point = point.field(key, value)
        write_api.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=point)
        logging.info("Debugdata sendt til InfluxDB.")
    except Exception as e:
        logging.error(f"Feil ved sending til InfluxDB: {e}")

# ----------------- Hovedprogram -----------------
def main():
    global running
    try:
        setup_ultrasound()
        bus = setup_compass()
        lidar_thread_obj = threading.Thread(target=lidar_thread)
        lidar_thread_obj.daemon = True
        lidar_thread_obj.start()
        
        move_forward()
        last_log_time = time.time()
        
        while True:
            compass_heading = read_compass(bus)
            cardinal = get_cardinal_direction(compass_heading)
            logging.info(f"Kompassretning: {compass_heading:.2f}� ({cardinal})")
            
            if check_ultrasound():
                stop_robot()     .�import RPi.GPIO as GPIO
import serial
import time
import threading
import math
import smbus
import numpy as np
import logging
from rplidar import RPLidar
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

# Konfigurer logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# ----------------- InfluxDB-innstillinger -----------------
INFLUX_URL = "https://eu-central-1-1.aws.cloud2.influxdata.com/"
INFLUX_TOKEN = "ZbCkxFHS5X0_9PG3YMeznY07WFCsMfaV7MMTSWyb7Ckq72zxMdVq2rB20ZdyxccfB1di9wQOBguZv70A9VEErA=="
INFLUX_ORG = "61778c39081df8c1"
INFLUX_BUCKET = "Lidarino"

client = InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG)
write_api = client.write_api(write_options=SYNCHRONOUS)

# ----------------- LIDAR-konfigurasjon -----------------
PORT_NAME = "/dev/ttyUSB0"  # Riktig port for LIDAR
lidar = RPLidar(PORT_NAME, baudrate=115200)

# ----------------- Serial til ESP32 -----------------
ESP_PORT = "/dev/ttyUSB1"  # Riktig port for ESP32
esp = serial.Serial(ESP_PORT, 115200, timeout=1)
time.sleep(2)  # La ESP32 starte opp

# ----------------- Parametere for hindringsdeteksjon -----------------
STOP_DISTANCE_LIDAR = 250       # mm
STOP_DISTANCE_ULTRASOUND = 15     # cm
SPEED = 100                       # mm/s (fremover)
FORWARD_DISTANCE_AFTER_TURN = 10  # mm
TIME_TO_MOVE_SHORT = FORWARD_DISTANCE_AFTER_TURN / SPEED  # sekunder
ROTATION_SPEED = 2                # (til ESP32-kommandoer)
ROTATION_TIME_90_DEG = 1.5        # Tid for en 90�-rotasjon

current_distance_lidar = 9999
running = True

# Global pose representert som en 3x3 SE2-matrise (identitets