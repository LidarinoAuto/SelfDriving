import RPi.GPIO as GPIO
import serial
import time
import threading
import random
from rplidar import RPLidar
import smbus

# ----------------- LIDAR-konfigurasjon -----------------
PORT_NAME = "/dev/ttyUSB1"
lidar = RPLidar(PORT_NAME, baudrate=115200)

# ----------------- Serial til ESP32 -----------------
ESP_PORT = "/dev/ttyUSB0"
esp = serial.Serial(ESP_PORT, 115200, timeout=1)
time.sleep(2)

# ----------------- Parametere -----------------
STOP_DISTANCE_LIDAR = 250  # mm
STOP_DISTANCE_ULTRASOUND = 100  # mm
SPEED = 100
FORWARD_DISTANCE_AFTER_TURN = 100
TIME_TO_MOVE_1M = FORWARD_DISTANCE_AFTER_TURN / SPEED
ROTATION_SPEED = 2
ROTATION_TIME_90_DEG = 1.5

current_distance_lidar = 9999
running = True

# ----------------- Ultralydsensor -----------------
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
    if duration >= 38000:
        return -1
    else:
        return (duration / 58) * 10  # mm

def check_front_ultrasound():
    for i in [0, 1]:
        d = les_avstand(trig_pins[i], echo_pins[i])
        if d > 0 and d < STOP_DISTANCE_ULTRASOUND:
            print(f"Ultralydsensor {i} oppdager hindring: {d:.0f} mm")
            return True
    return False

# ----------------- LIDAR-tr�d -----------------
def lidar_thread():
    global current_distance_lidar, running
    for scan in lidar.iter_scans():
        distances = []
        for measurement in scan:
            angle = measurement[1]
            distance = measurement[2]
            if abs(angle - 0) <= 30 or abs(angle - 360) <= 30:
                distances.append(distance)
        if distances:
            current_distance_lidar = min(distances)
        if not running:
            break

# ----------------- ESP32 Motorstyring -----------------
def send_command(command):
    print(f"Sender til ESP32: {command.strip()}")
    esp.write(command.encode())

def move_forward():
    print("Kj�rer fremover")
    send_command("100 0 0\n")

def stop_robot():
    print("Stopper roboten")
    send_command("0 0 0\n")

def move_forward_distance():
    send_command(f"{SPEED} 0 0\n")
    time.sleep(TIME_TO_MOVE_1M)
    send_command("0 0 0\n")

# ----------------- IMU -----------------
IMU_ADDRESS = 0x68
bus = smbus.SMBus(1)

def init_imu():
    bus.write_byte_data(IMU_ADDRESS, 0x6B, 0)
    time.sleep(0.1)

def read_word(adr):
    high = bus.read_byte_data(IMU_ADDRESS, adr)
    low = bus.read_byte_data(IMU_ADDRESS, adr + 1)
    return (high << 8) + low

def read_word_2c(adr):
    val = read_word(adr)
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

    stop_robot()
    print(f"Rotert: {integrated_angle:.1f} grader")

def choose_direction_with_lidar():
    scan_data = {"left": [], "right": [], "front": []}
    for scan in lidar.iter_scans(max_buf_meas=500):
        for measurement in scan:
            angle = measurement[1]
            distance = measurement[2]
            if 60 <= angle <= 120:
                scan_data["left"].append(distance)
            elif 240 <= angle <= 300:
                scan_data["right"].append(distance)
            elif angle <= 30 or angle >= 330:
                scan_data["front"].append(distance)
        if all(len(scan_data[key]) > 10 for key in scan_data):
            break

    avg_left = sum(scan_data["left"]) / len(scan_data["left"])
    avg_right = sum(scan_data["right"]) / len(scan_data["right"])
    avg_front = sum(scan_data["front"]) / len(scan_data["front"])

    print(f"LIDAR: Front: {avg_front:.1f}mm, Venstre: {avg_left:.1f}mm, H�yre: {avg_right:.1f}mm")

    if avg_left < 300 and avg_right < 300:
        print("Begge sider blokkert. Rygger.")
        send_command("-100 0 0\n")
        time.sleep(0.5)
        stop_robot()
        time.sleep(0.2)
        new_front_distances = []
        for scan in lidar.iter_scans(max_buf_meas=200):
            for m in scan:
                a = m[1]
                d = m[2]
                if a <= 30 or a >= 330:
                    new_front_distances.append(d)
            if len(new_front_distances) > 10:
                break

        new_avg_front = sum(new_front_distances) / len(new_front_distances)
        print(f"Ny front: {new_avg_front:.1f} mm")
        if new_avg_front > 400:
            print("Fritt foran. Fremover.")
            move_forward()
            return
        else:
            angle = random.choice([-90, 90])
            rotate_by_angle(angle)
            return

    if avg_left > avg_right:
        rotate_by_angle(-45)
    else:
        rotate_by_angle(45)

# ----------------- MAIN -----------------
try:
    setup_ultrasound()
    init_imu()
    lidar_thread_obj = threading.Thread(target=lidar_thread)
    lidar_thread_obj.daemon = True
    lidar_thread_obj.start()

    move_forward()

    while True:
        if check_front_ultrasound():
            stop_robot()
            time.sleep(1)
            choose_direction_with_lidar()
            move_forward_distance()
            move_forward()
            continue

        if current_distance_lidar <= STOP_DISTANCE_LIDAR:
            stop_robot()
            time.sleep(1)
            choose_direction_with_lidar()
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
