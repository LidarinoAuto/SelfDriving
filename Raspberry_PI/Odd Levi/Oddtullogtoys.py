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

# ----------------- Serial til ESP32 -----------------
ESP_PORT = "/dev/ttyUSB0"  # Riktig port for ESP32
esp = serial.Serial(ESP_PORT, 115200, timeout=1)
time.sleep(2)  # La ESP32 starte opp

# ----------------- Parametere for hindringsdeteksjon -----------------
STOP_DISTANCE_LIDAR = 250         # Lidar: 250 mm
STOP_DISTANCE_ULTRASOUND = 15       # Ultralyd: 15 cm
SPEED = 100                       # mm/s, fremoverhastighet
FORWARD_DISTANCE_AFTER_TURN = 10  # mm, kort strekning etter rotasjon
TIME_TO_MOVE_1M = FORWARD_DISTANCE_AFTER_TURN / SPEED  # sekunder
ROTATION_SPEED = 2                # Hovedrotasjonshastighet (brukes i kommandoer til ESP32)
# ROTATION_TIME_90_DEG defineres ikke lenger n�r vi bruker gyrosensor for kontroll

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
        distances = [measurement[2] for measurement in scan 
                     if abs(measurement[1] - 0) <= 30 or abs(measurement[1] - 360) <= 30]
        if distances:
            current_distance_lidar = min(distances)
        if not running:
            break

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
    return (high << 8) + low

def read_word_2c(adr):
    val = read_word(adr)
    return -((65535 - val) + 1) if val >= 0x8000 else val

def get_gyro_z():
    """
    Leser gyroskopdata for z-aksen (yaw) og returnerer grader per sekund.
    """
    gyro_z = read_word_2c(0x47)
    return gyro_z / 131.0

def rotate_by_gyro(target_angle):
    """
    Roterer roboten ved hjelp av gyroskopdata til � oppn� en integrert rotasjon p� target_angle grader.
    For � unng� oversving, reduseres hastigheten n�r roboten n�rmer seg m�let.
    Sanntidsvisning av integrert vinkel vises underveis.
    """
    integrated_angle = 0.0
    last_time = time.time()
    update_interval = 0.1
    last_update = time.time()
    
    # Start med full hastighet
    speed_command = ROTATION_SPEED
    print(f"Starter rotasjon for � oppn� {target_angle}�")
    send_command(f"0 0 {-speed_command}\n")
    
    while abs(integrated_angle) < abs(target_angle):
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        integrated_angle += get_gyro_z() * dt
        
        # N�r vi er n�r m�l, reduser hastigheten for � unng� oversving
        if abs(target_angle) - abs(integrated_angle) < 10:
            # Senkere hastighet: minimum 1 for � sikre fortsatt rotasjon
            slow_speed = max(1, speed_command / 2)
            send_command(f"0 0 {-slow_speed}\n")
        
        if current_time - last_update >= update_interval:
            print(f"Roterer... integrert vinkel: {integrated_angle:.2f}�")
            last_update = current_time

        time.sleep(0.005)

    send_command("0 0 0\n")
    print(f"Fullf�rt rotasjon, totalt {integrated_angle:.2f} grader")

def rotate_until_free():
    """
    Roterer roboten med 90�-inkrement til en fri retning oppdages (ingen hindring if�lge ultralydsensorene).
    Etter hver 90�-rotasjon venter funksjonen en kort stund for at sensorene skal stabilisere seg,
    deretter sjekkes med check_ultrasound(). Dersom hindring fortsatt oppdages, roteres ytterligere 90�.
    Returnerer True dersom en fri retning er funnet, ellers False etter en full 360�-rotasjon.
    """
    rotations = 0
    while rotations < 4:
        print(f"Rotasjon {rotations+1}: Roterer 90 grader for � sjekke for fri retning.")
        rotate_by_gyro(90)
        rotations += 1
        # La sensorene stabilisere seg litt etter rotasjon
        time.sleep(0.5)
        if not check_ultrasound():
            print("Fri retning funnet!")
            return True
        else:
            print("Hindring fortsatt. Fortsetter � rotere 90 grader til.")
    print("Ingen fri retning funnet etter 360 grader.")
    return False

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
    init_imu()  # Initialiserer IMU for rotasjonsm�ling
    lidar_thread_obj = threading.Thread(target=lidar_thread, daemon=True)
    lidar_thread_obj.start()

    # Start med � kj�re fremover
    move_forward()

    while True:
        # Dersom ultralydsensorene registrerer en hindring, stopp roboten
        if check_ultrasound():
            stop_robot()
            time.sleep(1)
            if rotate_until_free():
                move_forward_distance()
                move_forward()
            else:
                print("Ingen fri retning funnet, stopper roboten.")
                stop_robot()
                break
            continue

        # Dersom LIDAR oppdager hindring i f�rerfeltet, stopp og let etter fri retning
        print(f"LIDAR-avstand: {current_distance_lidar:.1f} mm")
        if current_distance_lidar <= STOP_DISTANCE_LIDAR:
            stop_robot()
            time.sleep(1)
            if rotate_until_free():
                move_forward_distance()
                move_forward()
            else:
                print("Ingen fri retning funnet, stopper roboten.")
                stop_robot()
                break

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
