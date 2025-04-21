import RPi.GPIO as GPIO
import serial
import time
import threading
import random
from rplidar import RPLidar
import smbus
import math

# ----------------- LIDAR-konfigurasjon -----------------
PORT_NAME = "/dev/rplidar"
lidar = RPLidar(PORT_NAME, baudrate=115200)

# ----------------- Serial til ESP32 -----------------
ESP_PORT = "/dev/esp32"
esp = serial.Serial(ESP_PORT, 115200, timeout=1)
time.sleep(2)

# ----------------- Parametere for hindringsdeteksjon -----------------
STOP_DISTANCE_LIDAR = 250
STOP_DISTANCE_ULTRASOUND = 15
SPEED = 100
FORWARD_DISTANCE_AFTER_TURN = 10
TIME_TO_MOVE_1M = FORWARD_DISTANCE_AFTER_TURN / SPEED
ROTATION_SPEED = 2
ROTATION_TIME_90_DEG = 1.5

current_distance_lidar = 9999
running = True

# ----------------- Ultralydsensor-konfigurasjon -----------------
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

    duration = (pulse_end - pulse_start) * 1e6
    return -1 if duration >= 38000 else duration / 58.0

def get_best_ultrasound_direction():
    """
    Leser kun de fremre ultralydsensorene (front_left og front_right)
    og returnerer (best_index, best_distance).
    """
    sensor_names = ["front_left", "front_right"]
    front_trig_pins = trig_pins[:2]
    front_echo_pins = echo_pins[:2]

    distances = []
    for i in range(len(front_trig_pins)):
        d = les_avstand(front_trig_pins[i], front_echo_pins[i])
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
    return any(0 < les_avstand(trig_pins[i], echo_pins[i]) < STOP_DISTANCE_ULTRASOUND for i in range(len(trig_pins)))

# ----------------- LIDAR-tr�d -----------------
def lidar_thread():
    global current_distance_lidar, running
    for scan in lidar.iter_scans():
        distances = [measurement[2] for measurement in scan if abs(measurement[1] - 0) <= 30 or abs(measurement[1] - 360) <= 30]
        if distances:
            current_distance_lidar = min(distances)
        if not running:
            break

# ----------------- Motorstyring -----------------
def send_command(command):
    print(f"Sender til ESP32: {command.strip()}")
    esp.write(command.encode())

# ----------------- IMU-konfigurasjon -----------------
IMU_ADDRESS = 0x68
bus = smbus.SMBus(1)

def init_imu():
    bus.write_byte_data(IMU_ADDRESS, 0x6B, 0)
    time.sleep(0.1)

def read_word_2c(adr):
    val = (bus.read_byte_data(IMU_ADDRESS, adr) << 8) + bus.read_byte_data(IMU_ADDRESS, adr + 1)
    return -((65535 - val) + 1) if val >= 0x8000 else val

def get_gyro_z():
    return read_word_2c(0x47) / 131.0

def rotate_by_angle(angle):
    rotation_time = (abs(angle) / 90) * ROTATION_TIME_90_DEG
    send_command(f"0 0 {ROTATION_SPEED if angle < 0 else -ROTATION_SPEED}\n")
    
    start_time = time.time()
    last_time = start_time
    integrated_angle = 0.0

    while time.time() < start_time + rotation_time:
        dt = time.time() - last_time
        last_time = time.time()
        integrated_angle += get_gyro_z() * dt
        time.sleep(0.01)

    send_command("0 0 0\n")
    print(f"Integrert rotasjon m�lt av gyroskop: {integrated_angle:.2f} grader")

def choose_direction_and_rotate():
    best_index, best_distance = get_best_ultrasound_direction()
    angle = -30 if best_index == 0 else 30
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
    init_imu()
    lidar_thread_obj = threading.Thread(target=lidar_thread, daemon=True)
    lidar_thread_obj.start()
    
    move_forward()

    while True:
        if check_ultrasound():
            stop_robot()
            time.sleep(1)
            choose_direction_and_rotate()
            move_forward_distance()
            move_forward()
            continue
        
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
    stop_robot()

finally:
    running = False
    lidar.stop()
    lidar.disconnect()
    esp.close()
    GPIO.cleanup()
