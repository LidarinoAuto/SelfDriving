import RPi.GPIO as GPIO
import serial
import time
import threading
from rplidar import RPLidar
import smbus
import math

# ----------------- LIDAR-konfigurasjon -----------------
PORT_NAME = "/dev/rplidar"      # Riktig port for LIDAR
lidar = RPLidar(PORT_NAME, baudrate=115200)

# ----------------- Serial til ESP32 -----------------
ESP_PORT = "/dev/esp32"       # Riktig port for ESP32
esp = serial.Serial(ESP_PORT, 115200, timeout=1)
time.sleep(2)                   # La ESP32 starte opp

# ----------------- Parametere for hindringsdeteksjon -----------------
STOP_DISTANCE_LIDAR = 250       # Lidar: 250 mm
STOP_DISTANCE_ULTRASOUND = 15     # Ultralyd: 15 cm
SPEED = 100                     # mm/s, fremoverhastighet
FORWARD_DISTANCE_AFTER_TURN = 10  # mm, kort strekning etter rotasjon
TIME_TO_MOVE_1M = FORWARD_DISTANCE_AFTER_TURN / SPEED  # sekunder
ROTATION_SPEED = 2              # Rotasjonshastighet

current_distance_lidar = 9999
running = True

# Global buffer for flere LIDAR-skanninger
lidar_buffer = []  # Vi samler f.eks. de siste 5 skanningene

# ----------------- Ultralydsensor-konfigurasjon -----------------
trig_pins_front = [9, 7]        # Fremre sensorer
echo_pins_front = [8, 6]
trig_pins_back = [23, 10]       # Bakre sensorer
echo_pins_back = [24, 11]

all_trig_pins = trig_pins_front + trig_pins_back
all_echo_pins = echo_pins_front + echo_pins_back

def setup_ultrasound():
    GPIO.setmode(GPIO.BCM)
    for trig in all_trig_pins:
        GPIO.setup(trig, GPIO.OUT)
        GPIO.output(trig, False)
    for echo in all_echo_pins:
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

def check_ultrasound_all():
    for i in range(len(all_trig_pins)):
        d = les_avstand(all_trig_pins[i], all_echo_pins[i])
        if d > 0 and d < STOP_DISTANCE_ULTRASOUND:
            print(f"Ultralydsensor {i} oppdager hindring: {d:.2f} cm")
            return True
    return False

# ----------------- LIDAR-tr�d -----------------
def lidar_thread():
    global current_distance_lidar, running, lidar_buffer
    for scan in lidar.iter_scans():
        distances = [measurement[2] for measurement in scan 
                     if abs(measurement[1] - 0) <= 30 or abs(measurement[1] - 360) <= 30]
        if distances:
            min_distance = min(distances)
            current_distance_lidar = min_distance
            lidar_buffer.append(min_distance)
            if len(lidar_buffer) > 5:
                lidar_buffer.pop(0)
        if not running:
            break

def get_median_lidar_reading():
    if not lidar_buffer:
        return float('inf')
    sorted_buffer = sorted(lidar_buffer)
    n = len(sorted_buffer)
    return sorted_buffer[n // 2] if n % 2 == 1 else (sorted_buffer[n // 2 - 1] + sorted_buffer[n // 2]) / 2

def lidar_scan_for_open_path():
    global current_distance_lidar
    print("Utf�rer LIDAR-skanning for fri vei...")
    time.sleep(1)
    stable_distance = get_median_lidar_reading()
    print(f"Stabil LIDAR-avlesning (median): {stable_distance:.1f} mm")
    return stable_distance > STOP_DISTANCE_LIDAR

# ----------------- Motorstyring (via ESP32) -----------------
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
    print("Kj�rer et kort stykke fremover etter rotasjon")
    send_command(f"{SPEED} 0 0\n")
    time.sleep(TIME_TO_MOVE_1M)
    send_command("0 0 0\n")

# ----------------- IMU-konfigurasjon (MPU6050) -----------------
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
    gyro_z = read_word_2c(0x47)
    return gyro_z / 131.0

def rotate_by_gyro(target_angle):
    integrated_angle = 0.0
    last_time = time.time()
    update_interval = 0.1
    last_update = time.time()
    print(f"Starter rotasjon for � oppn� {target_angle}�")
    speed_command = ROTATION_SPEED
    send_command(f"0 0 {-speed_command}\n")

    while abs(integrated_angle) < abs(target_angle):
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        integrated_angle += get_gyro_z() * dt

        if abs(target_angle) - abs(integrated_angle) < 10:
            slow_speed = max(1, speed_command // 2)
            send_command(f"0 0 {-slow_speed}\n")

        if current_time - last_update >= update_interval:
            print(f"Roterer... integrert vinkel: {integrated_angle:.2f}�")
            last_update = current_time

        time.sleep(0.005)

    send_command("0 0 0\n")
    print(f"Fullf�rt rotasjon, totalt {integrated_angle:.2f} grader")

def rotate_until_free():
    rotations = 0
    while rotations < 4:
        print(f"Rotasjon {rotations+1}: Roterer 90 grader for � sjekke for fri retning.")
        rotate_by_gyro(90)
        rotations += 1
        time.sleep(1)
        if lidar_scan_for_open_path():
            print("Fri retning funnet!")
            return True
        else:
            print("Hindring registrert under LIDAR-skanning, roterer ytterligere 90 grader.")
    print("Ingen fri retning funnet etter 360 grader.")
    return False

# ----------------- Kompass (HMC5883L) -----------------
HMC5883L_ADDRESS = 0x0d

def init_compass():
    bus.write_byte_data(HMC5883L_ADDRESS, 0x00, 0x70)
    bus.write_byte_data(HMC5883L_ADDRESS, 0x01, 0xA0)
    bus.write_byte_data(HMC5883L_ADDRESS, 0x02, 0x00)

def read_compass():
    data = bus.read_i2c_block_data(HMC5883L_ADDRESS, 0x03, 6)
    x = data[0] << 8 | data[1]
    z = data[2] << 8 | data[3]
    y = data[4] << 8 | data[5]
    x = x - 65536 if x > 32767 else x
    y = y - 65536 if y > 32767 else y
    z = z - 65536 if z > 32767 else z
    heading_rad = math.atan2(y, x)
    heading_deg = math.degrees(heading_rad)
    if heading_deg < 0:
        heading_deg += 360
    return heading_deg

def get_compass_direction(deg):
    dirs = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
    index = int((deg + 22.5) % 360 // 45)
    return dirs[index]

# ----------------- HOVEDPROGRAM -----------------
try:
    setup_ultrasound()
    init_imu()
    init_compass()
    lidar_thread_obj = threading.Thread(target=lidar_thread, daemon=True)
    lidar_thread_obj.start()

    move_forward()

    while True:
        heading = read_compass()
        direction = get_compass_direction(heading)
        print(f"Kompasskurs: {heading:.1f}� ({direction})")  # Kompassretning i grader og tekst

        if check_ultrasound_all():
            stop_robot()
            time.sleep(1)
            print("Ultralyd oppdager hindring, roterer for � finne ny retning.")
            rotate_until_free()
            move_forward_distance()
            move_forward()
            continue

        stable_distance = get_median_lidar_reading()
        print(f"LIDAR stabil avlesning (median): {stable_distance:.1f} mm")
        if stable_distance <= STOP_DISTANCE_LIDAR:
            stop_robot()
            time.sleep(1)
            print("LIDAR registrerer hindring, roterer for � finne fri retning.")
            rotate_until_free()
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
