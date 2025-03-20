import RPi.GPIO as GPIO
import serial
import time
import threading
from rplidar import RPLidar

# ----------------- LIDAR-konfigurasjon -----------------
PORT_NAME = "/dev/ttyUSB1"  # Riktig port for LIDAR
lidar = RPLidar(PORT_NAME, baudrate=115200)

# ----------------- Serial til ESP32 -----------------
ESP_PORT = "/dev/ttyUSB0"  # Riktig port for ESP32
esp = serial.Serial(ESP_PORT, 115200, timeout=1)
time.sleep(2)  # La ESP32 starte opp

# ----------------- Parametere -----------------
STOP_DISTANCE_LIDAR = 500       
STOP_DISTANCE_ULTRASOUND = 100  
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

def check_ultrasound():
    for i in range(len(trig_pins)):
        d = les_avstand(trig_pins[i], echo_pins[i])
        if d > 0 and d < STOP_DISTANCE_ULTRASOUND:
            print(f"Ultralydsensor {i} oppdager hindring: {d:.2f} cm")
            return True
    return False

# ----------------- LIDAR-tr�d -----------------
def lidar_thread():
    global current_distance_lidar, running
    try:
        for scan in lidar.iter_scans():
            if not running:
                break  
            distances = []
            for measurement in scan:
                angle = measurement[1]  
                distance = measurement[2]  
                if abs(angle - 0) <= 30 or abs(angle - 360) <= 30:
                    distances.append(distance)
            if distances:
                current_distance_lidar = min(distances)
    except serial.SerialException:
        print("LIDAR serial error: Device disconnected")
    except Exception as e:
        print(f"Ukjent feil i LIDAR-tr�den: {e}")
    finally:
        print("LIDAR-tr�den avsluttes...")

# ----------------- Motorstyring -----------------
def send_command(command):
    print(f"Sender til ESP32: {command.strip()}")
    esp.write(command.encode())

def rotate_by_angle(angle):
    rotation_time = (abs(angle) / 90) * ROTATION_TIME_90_DEG
    send_command(f"0 0 {ROTATION_SPEED if angle < 0 else -ROTATION_SPEED}\n")
    time.sleep(rotation_time)
    send_command("0 0 0\n")

def move_forward():
    print("Kj�rer fremover")
    send_command("100 0 0\n")

def stop_robot():
    print("Stopper roboten")
    send_command("0 0 0\n")

def move_forward_distance():
    print("Kj�rer kort strekning fremover etter rotasjon")
    send_command(f"{SPEED} 0 0\n")
    time.sleep(TIME_TO_MOVE_1M)
    send_command("0 0 0\n")

# ----------------- LIDAR STOP FUNKSJON -----------------
def stop_lidar():
    """ Stopper LIDAR-motoren fysisk f�r frakobling. """
    print("Stopper LIDAR om 5 sekunder...")
    time.sleep(5)  # **Vent 5 sekunder f�r vi stopper LIDAR**
    try:
        lidar.stop_motor()  # **Viktig for � stoppe rotasjon!**
        lidar.stop()
        lidar.disconnect()
        print("? LIDAR er stoppet og frakoblet.")
    except Exception as e:
        print(f"?? Feil ved stopp av LIDAR: {e}")

# ----------------- HOVEDPROGRAM -----------------
try:
    setup_ultrasound()
    lidar_thread_obj = threading.Thread(target=lidar_thread, daemon=True)
    lidar_thread_obj.start()
    
    move_forward()
    
    while True:
        if check_ultrasound():
            stop_robot()
            time.sleep(1)
            rotate_by_angle(90)
            move_forward_distance()
            move_forward()
            continue
        
        print(f"LIDAR-avstand: {current_distance_lidar:.1f} mm")
        if current_distance_lidar <= STOP_DISTANCE_LIDAR:
            stop_robot()
            time.sleep(5)  # **Vent 5 sekunder f�r vi roterer**
            rotate_by_angle(90)
            move_forward_distance()
            move_forward()
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Avslutter programmet...")
    stop_robot()

    # ? **LIDAR stoppes kun n�r programmet avsluttes!**
    stop_lidar()  

finally:
    running = False  
    
    print("Lukker ESP32-tilkobling...")
    esp.close()
    
    print("Rydder opp GPIO...")
    GPIO.cleanup()
    print("? Program avsluttet.")
