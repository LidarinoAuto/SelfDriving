import RPi.GPIO as GPIO
import time
import smbus
import logging
from rplidar import RPLidar

# GPIO-oppsett for ultralydsensorer
ULTRASOUND_PINS = {
    "front": (23, 24),
    "left": (17, 27),
    "right": (5, 6),
}

# LIDAR-init
def init_lidar(port):
    lidar = RPLidar(port)
    lidar.connect()
    lidar.start_motor()
    logging.info("LIDAR startet p� port %s", port)
    return lidar

def lidar_thread(lidar, current_distance_lidar, running_flag):
    try:
        for scan in lidar.iter_scans():
            if not running_flag[0]:
                break
            # Finn minste avstand fra skanning
            closest = min(scan, key=lambda x: x[2]) if scan else (None, None, 9999)
            current_distance_lidar[0] = closest[2]
    except Exception as e:
        logging.error(f"Lidar_thread error: {e}")

# Ultrasound-oppsett

def setup_ultrasound():
    GPIO.setmode(GPIO.BCM)
    for trig, echo in ULTRASOUND_PINS.values():
        GPIO.setup(trig, GPIO.OUT)
        GPIO.setup(echo, GPIO.IN)
        GPIO.output(trig, False)
    time.sleep(0.5)

def measure_distance(trig, echo):
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    start, stop = time.time(), time.time()

    timeout = start + 0.05  # maks ventetid 50ms
    while GPIO.input(echo) == 0 and time.time() < timeout:
        start = time.time()
    while GPIO.input(echo) == 1 and time.time() < timeout:
        stop = time.time()

    elapsed = stop - start
    distance = (elapsed * 34300) / 2
    return round(distance, 2)

def check_ultrasound(threshold_cm):
    for name, (trig, echo) in ULTRASOUND_PINS.items():
        dist = measure_distance(trig, echo)
        if dist < threshold_cm:
            logging.info(f"Hinder oppdaget av {name} sensor: {dist:.1f} cm")
            return True
    return False

def read_all_ultrasound():
    readings = {}
    for name, (trig, echo) in ULTRASOUND_PINS.items():
        readings[name] = measure_distance(trig, echo)
    return readings

def get_best_ultrasound_direction():
    readings = read_all_ultrasound()
    best = max(readings.items(), key=lambda x: x[1])
    return best[0], best[1]

# Kompass (MPU-6050 eller HMC5883L)
I2C_ADDRESS = 0x1E  # Juster etter kompassmodellen

def setup_compass():
    bus = smbus.SMBus(1)
    try:
        # Eksempel for HMC5883L
        bus.write_byte_data(I2C_ADDRESS, 0x00, 0x70)
        bus.write_byte_data(I2C_ADDRESS, 0x01, 0xA0)
        bus.write_byte_data(I2C_ADDRESS, 0x02, 0x00)
    except Exception as e:
        logging.error(f"Feil ved oppsett av kompass: {e}")
    return bus

def read_compass(bus):
    try:
        data = bus.read_i2c_block_data(I2C_ADDRESS, 0x03, 6)
        x = data[0] << 8 | data[1]
        z = data[2] << 8 | data[3]
        y = data[4] << 8 | data[5]
        heading = math.atan2(y, x) * 180 / math.pi
        if heading < 0:
            heading += 360
        return heading
    except Exception as e:
        logging.error(f"Feil ved lesing fra kompass: {e}")
        return 0.0

def get_cardinal_direction(heading):
    dirs = ['Nord', 'Nord�st', '�st', 'S�r�st', 'S�r', 'S�rvest', 'Vest', 'Nordvest']
    ix = int((heading + 22.5) // 45) % 8
    return dirs[ix]
