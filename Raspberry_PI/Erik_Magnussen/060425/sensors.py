import RPi.GPIO as GPIO
import time
import smbus
import math
import logging
from rplidar import RPLidar

# Konfigurasjon for ultralydsensorer
SENSORS = {
    "FrontVenstre": {"trig": 9, "echo": 8},
    "FrontHoyre": {"trig": 7, "echo": 6},
    "BakVenstre": {"trig": 23, "echo": 24},
    "BakHoyre": {"trig": 10, "echo": 11}
}

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
    """Leser alle ultralydsensorene og returnerer en dict med avstander (i cm)."""
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

def check_ultrasound(stop_distance):
    """Returnerer True dersom minst �n sensor m�ler en avstand under 'stop_distance' (cm)."""
    data = read_all_ultrasound()
    for navn, avstand in data.items():
        if avstand is not None and avstand < stop_distance:
            logging.info(f"{navn} oppdager hindring: {avstand:.2f} cm")
            return True
    return False

# LIDAR-funksjoner
def init_lidar(port, baudrate=115200):
    return RPLidar(port, baudrate=baudrate)

def lidar_thread(lidar, current_distance, running_flag):
    """
    Leser LIDAR-skanninger og oppdaterer current_distance (liste med ett element)
    mens running_flag[0] er True.
    """
    for scan in lidar.iter_scans():
        distances = [m[2] for m in scan if abs(m[1] - 0) <= 30 or abs(m[1] - 360) <= 30]
        if distances:
            current_distance[0] = min(distances)
        if not running_flag[0]:
            break

# Kompass-funksjoner
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
    if heading < 0:
        heading += 360
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
