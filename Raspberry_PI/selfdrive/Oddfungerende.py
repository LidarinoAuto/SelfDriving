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
# Under normal kj�ring benyttes kun de fremre sensorene.
trig_pins_front = [9, 7]        # Fremre sensorer
echo_pins_front = [8, 6]
# Bakre sensorer defineres her (brukes ved full blokkeringssjekk eller utvidelse)
trig_pins_back = [23, 10]
echo_pins_back = [24, 11]

def setup_ultrasound():
    GPIO.setmode(GPIO.BCM)
    # Sett opp kun fremre sensorer for normal kj�ring:
    for trig in trig_pins_front:
        GPIO.setup(trig, GPIO.OUT)
        GPIO.output(trig, False)
    for echo in echo_pins_front:
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

    duration = (pulse_end - pulse_start) * 1e6  # konverterer til mikrosekunder
    return -1 if duration >= 38000 else duration / 58.0  # Avstand i cm

def check_ultrasound_front():
    """
    Sjekker kun de fremre ultralydsensorene for hindringer.
    """
    for i in range(len(trig_pins_front)):
        d = les_avstand(trig_pins_front[i], echo_pins_front[i])
        if d > 0 and d < STOP_DISTANCE_ULTRASOUND:
            print(f"Fremre ultralydsensor {i} oppdager hindring: {d:.2f} cm")
            return True
    return False

# ----------------- LIDAR-tr�d -----------------
def lidar_thread():
    global current_distance_lidar, running, lidar_buffer
    for scan in lidar.iter_scans():
        # Vi vurderer her bare m�linger i et snevert fremoverfelt (�30�)
        distances = [measurement[2] for measurement in scan 
                     if abs(measurement[1] - 0) <= 30 or abs(measurement[1] - 360) <= 30]
        if distances:
            min_distance = min(distances)
            current_distance_lidar = min_distance
            # Legg til den nye m�lingen i bufferen
            lidar_buffer.append(min_distance)
            if len(lidar_buffer) > 5:
                lidar_buffer.pop(0)
        if not running:
            break

def get_median_lidar_reading():
    """
    Beregner medianverdien av de siste LIDAR-m�lingene lagret i lidar_buffer.
    """
    if not lidar_buffer:
        return float('inf')
    sorted_buffer = sorted(lidar_buffer)
    n = len(sorted_buffer)
    if n % 2 == 1:
        return sorted_buffer[n // 2]
    else:
        return (sorted_buffer[n // 2 - 1] + sorted_buffer[n // 2]) / 2

def lidar_scan_for_open_path():
    """
    Utf�rer en LIDAR-skanning for � finne en fri retning.
    Basert p� medianverdien av de siste skanningene,
    tolkes en verdi over STOP_DISTANCE_LIDAR som en �pen vei.
    """
    global current_distance_lidar
    print("Utf�rer LIDAR-skanning for fri vei...")
    time.sleep(1)  # Kort pause slik at LIDAR-data stabiliseres
    stable_distance = get_median_lidar_reading()
    print(f"Stabil LIDAR-avlesning (median): {stable_distance:.1f} mm")
    if stable_distance > STOP_DISTANCE_LIDAR:
        return True
    return False

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
    """
    Roterer roboten ved hjelp av gyroskopdata til � oppn� en integrert rotasjon p� target_angle grader.
    Integrert vinkel vises i sanntid, og hastigheten reduseres n�r vi n�rmer oss m�l for � unng� oversving.
    """
    integrated_angle = 0.0
    last_time = time.time()
    update_interval = 0.1
    last_update = time.time()
    
    print(f"Starter rotasjon for � oppn� {target_angle}�")
    speed_command = ROTATION_SPEED
    # Anta at negativ hastighet roterer til h�yre.
    send_command(f"0 0 {-speed_command}\n")
    
    while abs(integrated_angle) < abs(target_angle):
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        integrated_angle += get_gyro_z() * dt
        
        # Reduser hastigheten n�r vi n�rmer oss m�l
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
    """
    Roterer roboten med 90�-inkrement til LIDAR-skanning bekrefter at veien er fri.
    Etter hver 90�-rotasjon venter systemet ca. 1 sekund f�r en ny skanning.
    N�r en fri retning er funnet, vil roboten fortsette � kj�re.
    """
    rotations = 0
    while rotations < 4:
        print(f"Rotasjon {rotations+1}: Roterer 90 grader for � sjekke for fri retning.")
        rotate_by_gyro(90)
        rotations += 1
        time.sleep(1)  # 1 sekund delay etter hver rotasjon
        if lidar_scan_for_open_path():
            print("Fri retning funnet!")
            return True
        else:
            print("Hindring registrert under LIDAR-skanning, roterer ytterligere 90 grader.")
    print("Ingen fri retning funnet etter 360 grader.")
    return False

# ----------------- HOVEDPROGRAM -----------------
try:
    setup_ultrasound()         # Sett opp kun de fremre sensorene
    init_imu()                 # Initialiserer IMU for rotasjonsm�ling
    lidar_thread_obj = threading.Thread(target=lidar_thread, daemon=True)
    lidar_thread_obj.start()

    # Start med � kj�re framover
    move_forward()

    while True:
        # Under normal kj�ring sjekkes kun de fremre ultralydsensorene.
        if check_ultrasound_front():
            stop_robot()
            time.sleep(1)
            print("Fremre sensorene oppdager hindring, roterer for � finne ny retning.")
            rotate_until_free()
            move_forward_distance()
            move_forward()
            continue
        
        # Bruk den stabile LIDAR-avlesningen (median) for � sjekkehindringer
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
