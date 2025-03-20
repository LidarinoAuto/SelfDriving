import RPi.GPIO as GPIO
import serial
import time
import threading
from rplidar import RPLidar

# ----------------- Configuration -----------------
PORT_NAME = "/dev/ttyUSB0"
lidar = RPLidar(PORT_NAME, baudrate=115200)

ESP_PORT = "/dev/ttyUSB1"
esp = serial.Serial(ESP_PORT, 115200, timeout=1)
time.sleep(2)

STOP_DISTANCE_LIDAR = 250  # mm
STOP_DISTANCE_ULTRASOUND = 15  # cm
SPEED = 100  # mm/s

current_distance_lidar = 9999
running = True

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

def read_distance(trig, echo):
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
        return duration / 58.0

def get_best_direction():
    distances = []
    for i in range(len(trig_pins)):
        d = read_distance(trig_pins[i], echo_pins[i])
        distances.append(d if d > 0 else 0)
    best_index = distances.index(max(distances))
    return best_index, max(distances)

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

def send_command(command):
    esp.write(command.encode())

def adjust_direction(best_index):
    if best_index == 0:  # Front-left sensor
        send_command("50 0 0\n")  # Slight turn to the right
    elif best_index == 1:  # Front-right sensor
        send_command("0 50 0\n")  # Slight turn to the left
    elif best_index == 2:  # Back-left sensor
        send_command("50 -50 0\n")  # Reverse and turn to the right
    elif best_index == 3:  # Back-right sensor
        send_command("-50 50 0\n")  # Reverse and turn to the left

def move_forward():
    send_command(f"{SPEED} 0 0\n")

def stop_robot():
    send_command("0 0 0\n")

# ----------------- Main Program -----------------
try:
    setup_ultrasound()
    lidar_thread_obj = threading.Thread(target=lidar_thread)
    lidar_thread_obj.daemon = True
    lidar_thread_obj.start()

    move_forward()

    while True:
        if current_distance_lidar <= STOP_DISTANCE_LIDAR:
            stop_robot()
            _, best_distance = get_best_direction()
            if best_distance > 0:
                adjust_direction(_)
            else:
                print("No clear path detected!")
            move_forward()
        
        time.sleep(0.1)

except KeyboardInterrupt:
    stop_robot()

finally:
    running = False
    lidar.stop()
    lidar.disconnect()
    esp.close()
    GPIO.cleanup()
