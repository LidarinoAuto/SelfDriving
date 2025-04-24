import RPi.GPIO as GPIO
import serial
import time
import threading
from rplidar import RPLidar
import smbus
import math
import numpy as np

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
FORWARD_DISTANCE_AFTER_TURN = 200  # mm, kort strekning etter rotasjon
TIME_TO_MOVE_1M = FORWARD_DISTANCE_AFTER_TURN / SPEED  # sekunder
ROTATION_SPEED = 2              # Rotasjonshastighet

# ----------------- Navigasjonsparametere -----------------
# Vinkelgrenser for hindringsskanning
SCAN_ANGLES = {
    'front': (-30, 30),        # Fremover (�30�)
    'left': (30, 90),          # Venstre (30� til 90�)
    'right': (-90, -30),       # H�yre (-90� til -30�)
    'back': (150, 210)         # Bak (150� til 210�)
}

# ----------------- Kompass-konfigurasjon (QMC5883L) -----------------
QMC5883L_ADDRESS = 0x0d
QMC5883L_CTRL1 = 0x09
QMC5883L_SET_RESET = 0x0B
QMC5883L_DATA = 0x00

def init_compass():
    """Initialiserer QMC5883L med riktige innstillinger."""
    try:
        bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_SET_RESET, 0x01)
        time.sleep(0.1)
        bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_CTRL1, 0b00011101)
        time.sleep(0.1)
        print("QMC5883L kompass initialisert vellykket.")
    except Exception as e:
        print(f"Feil under initialisering av QMC5883L: {e}")

def read_compass():
    """
    Leser r�data fra QMC5883L og beregner heading i grader.
    Returnerer heading i grader.
    """
    try:
        data = bus.read_i2c_block_data(QMC5883L_ADDRESS, QMC5883L_DATA, 6)

        x_raw = (data[1] << 8) | data[0]
        y_raw = (data[3] << 8) | data[2]
        z_raw = (data[5] << 8) | data[4]

        x = x_raw - 65536 if x_raw > 32767 else x_raw
        y = y_raw - 65536 if y_raw > 32767 else y_raw
        z = z_raw - 65536 if z_raw > 32767 else z_raw

        heading_rad = math.atan2(y, x)
        heading_deg = math.degrees(heading_rad)

        if heading_deg < 0:
            heading_deg += 360

        return heading_deg
    except Exception as e:
        print(f"Feil under lesing av QMC5883L: {e}")
        return -1

def get_compass_direction(deg):
    """Konverterer grader til en kardinal retning (N, NE, E, osv.)."""
    dirs = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 
            'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW']
    if deg < 0 or deg > 360:
        return "Ukjent"
    return dirs[int((deg + 11.25) % 360 // 22.5)]

current_distance_lidar = 9999
original_heading = 0            # Opprinnelig retning i grader
current_heading = 0             # N�v�rende retning (fra gyroskop)
true_heading = 0                # Faktisk retning fra kompass
heading_adjusted = False        # Flagg for � angi om vi har avveket fra original rute
running = True

# Global variabler for LIDAR og retningsdata
lidar_buffer = {}               # Dictionary med vinkel som n�kkel og liste av avstander som verdi
lidar_data = {}                 # Siste skanning organisert etter vinkel
compass_readings = []           # Buffer for kompassm�linger (for st�yreduksjon)

# ----------------- Ultralydsensor-konfigurasjon -----------------
# Under normal kj�ring benyttes kun de fremre sensorene.
trig_pins_front = [9, 7]        # Fremre sensorer
echo_pins_front = [8, 6]
# Bakre sensorer defineres her (brukes ved full blokkeringssjekk eller utvidelse)
trig_pins_side = [23, 10]       # Venstre og h�yre sensorer
echo_pins_side = [24, 11]

def setup_ultrasound():
    GPIO.setmode(GPIO.BCM)
    # Sett opp fremre sensorer:
    for trig in trig_pins_front + trig_pins_side:
        GPIO.setup(trig, GPIO.OUT)
        GPIO.output(trig, False)
    for echo in echo_pins_front + echo_pins_side:
        GPIO.setup(echo, GPIO.IN)
    time.sleep(0.5)

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

def check_ultrasound_all():
    """
    Sjekker alle ultralydsensorer og returnerer avstander i en ordbok.
    """
    distances = {
        'front_left': les_avstand(trig_pins_front[0], echo_pins_front[0]),
        'front_right': les_avstand(trig_pins_front[1], echo_pins_front[1]),
        'side_left': les_avstand(trig_pins_side[0], echo_pins_side[0]),
        'side_right': les_avstand(trig_pins_side[1], echo_pins_side[1]),
    }
    return distances

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
    global current_distance_lidar, running, lidar_buffer, lidar_data
    
    for scan in lidar.iter_scans():
        if not running:
            break
            
        # Oppdater lidar_data med nyeste skanning
        current_scan = {int(measurement[1]): measurement[2] for measurement in scan}
        lidar_data.update(current_scan)
        
        # Filtrer ut m�linger i frontretning for hurtig hindringdeteksjon
        front_distances = []
        for angle, distance in current_scan.items():
            # Konverter alle vinkler til omr�det [-180, 180]
            norm_angle = ((angle + 180) % 360) - 180
            
            # Sjekk om vinkelen er innenfor "fremover" omr�de
            if SCAN_ANGLES['front'][0] <= norm_angle <= SCAN_ANGLES['front'][1]:
                front_distances.append(distance)
                
                # Oppbevar data for stabilitet (buffer for hver vinkel)
                if norm_angle not in lidar_buffer:
                    lidar_buffer[norm_angle] = []
                
                lidar_buffer[norm_angle].append(distance)
                if len(lidar_buffer[norm_angle]) > 5:
                    lidar_buffer[norm_angle].pop(0)
        
        # Oppdater current_distance_lidar med minimum avstand foran roboten
        if front_distances:
            current_distance_lidar = min(front_distances)

# ----------------- Kompass-tr�d -----------------
def compass_thread():
    global true_heading, compass_readings, running
    
    while running:
        heading = read_compass()
        if heading >= 0:  # Gyldig avlesning
            compass_readings.append(heading)
            if len(compass_readings) > 10:  # Behold bare de siste 10 m�lingene
                compass_readings.pop(0)
                
            # Beregn medianverdi for � redusere st�y
            true_heading = np.median(compass_readings)
            
        time.sleep(0.05)  # 20 Hz oppdatering

def get_median_distance_in_range(angle_min, angle_max):
    """
    Henter median avstand fra LIDAR-data innenfor et vinkelomr�de.
    Returnerer 9999 hvis ingen data er tilgjengelig.
    """
    distances = []
    
    for angle, distance in lidar_data.items():
        # Normaliser vinkelen til [-180, 180]
        norm_angle = ((angle + 180) % 360) - 180
        if angle_min <= norm_angle <= angle_max:
            distances.append(distance)
    
    if not distances:
        return 9999
    
    return np.median(distances)

def get_median_front_distance():
    """
    Beregner medianverdien av LIDAR-avstand i front-retning.
    """
    return get_median_distance_in_range(SCAN_ANGLES['front'][0], SCAN_ANGLES['front'][1])

def scan_for_best_direction():
    """
    Skanner med LIDAR i alle retninger for � finne den beste �pne veien.
    Returnerer den anbefalte rotasjonsvinkelen (i grader).
    Positiv verdi betyr roter mot venstre, negativ verdi betyr roter mot h�yre.
    """
    distances = {}
    
    # Sjekk avstand i alle definerte retninger
    for direction, (angle_min, angle_max) in SCAN_ANGLES.items():
        median_dist = get_median_distance_in_range(angle_min, angle_max)
        distances[direction] = median_dist
        print(f"Retning {direction}: {median_dist}mm")
    
    # Sjekk f�rst om opprinnelig retning er klar nok
    if distances['front'] > STOP_DISTANCE_LIDAR * 1.5:
        print("Frontruten er klar, fortsetter rett fram")
        return 0
    
    # Sjekk side-retningene og velg den med st�rst avstand
    if distances['left'] > distances['right']:
        if distances['left'] > STOP_DISTANCE_LIDAR * 1.2: 
            print("Venstre side har best �pning")
            return 90  # 90 grader til venstre
    else:
        if distances['right'] > STOP_DISTANCE_LIDAR * 1.2:
            print("H�yre side har best �pning")
            return -90  # 90 grader til h�yre
    
    # Hvis ingen gode alternativer, sjekk bakover
    if distances['back'] > STOP_DISTANCE_LIDAR * 1.2:
        print("Ingen god sidealternativ, snur 180 grader")
        return 180
    
    # Hvis alle retninger er blokkert, velg den med mest plass
    best_dir = max(     M�import RPi.GPIO as GPIO
import serial
import time
import threading
from rplidar import RPLidar
import smbus
import math
import numpy as np

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
FORWARD_DISTANCE_AFTER_TURN = 200  # mm, kort strekning etter rotasjon
TIME_TO_MOVE_1M = FORWARD_DISTANCE_AFTER_TURN / SPEED  # sekunder
ROTATION_SPEED = 2              # Rotasjonshastighet

# ----------------- Navigasjonsparametere -----------------
# Vinkelgrenser for hindringsskanning
SCAN_ANGLES = {
    'front': (-30, 30),        # Fremover (�30�)
    'left': (30, 90),          # Venstre (30� til 90�)
    'right': (-90, -30),       # H�yre (-90� til -30�)
    'back': (150, 210)         # Bak (150� til 210�)
}

# ----------------- Kompass-konfigurasjon (QMC5883L) -----------------
QMC5883L_ADDRESS = 0x0d
QMC5883L_CTRL1 = 0x09
QMC5883L_SET_RESET = 0x0B
QMC5883L_DATA = 0x00

def init_compass():
    """Initialiserer QMC5883L med riktige innstillinger."""
    try:
        bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_SET_RESET, 0x01)
        time.sleep(0.1)
        bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_CTRL1, 0b00011101)
        time.sleep(0.1)
        print("QMC5883L kompass initialisert vellykket.")
    except Exception as e:
        print(f"Feil under initialisering av QMC5883L: {e}")

def read_compass():
    """
    Leser r�data fra QMC5883L og beregner heading i grader.
    Returnerer heading i grader.
    """
    try:
        data = bus.read_i2c_block_data(QMC5883L_ADDRESS, QMC5883L_DATA, 6)

        x_raw = (data[1] << 8) | data[0]
        y_raw = (data[3] << 8) | data[2]
        z_raw = (data[5] << 8) | data[4]

        x = x_raw - 65536 if x_raw > 32767 else x_raw
        y = y_raw - 65536 if y_raw > 32767 else y_raw
        z = z_raw - 65536 if z_raw > 32767 else z_raw

        heading_rad = math.atan2(y, x)
        heading_deg = math.degrees(heading_rad)

        if heading_deg < 0:
            heading_deg += 360

        return heading_deg
    except Exception as e:
        print(f"Feil under lesing av QMC5883L: {e}")
        return -1

def get_compass_direction(deg):
    """Konverterer grader til en kardinal retning (N, NE, E, osv.)."""
    dirs = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 
            'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW']
    if deg < 0 or deg > 360:
        return "Ukjent"
    return dirs[int((deg + 11.25) % 360 // 22.5)]

current_distance_lidar = 9999
original_heading = 0            # Opprinnelig retning i grader
current_heading = 0             # N�v�rende retning (fra gyroskop)
true_heading = 0                # Faktisk retning fra kompass
heading_adjusted = False        # Flagg for � angi om vi har avveket fra original rute
running = True

# Global variabler for LIDAR og retningsdata
lidar_buffer = {}               # Dictionary med vinkel som n�kkel og liste av avstander som verdi
lidar_data = {}                 # Siste skanning organisert etter vinkel
compass_readings = []           # Buffer for kompassm�linger (for st�yreduksjon)

# ----------------- Ultralydsensor-konfigurasjon -----------------
# Under normal kj�ring benyttes kun de fremre sensorene.
trig_pins_front = [9, 7]        # Fremre sensorer
echo_pins_front = [8, 6]
# Bakre sensorer defineres her (brukes ved full blokkeringssjekk eller utvidelse)
trig_pins_side = [23, 10]       # Venstre og h�yre sensorer
echo_pins_side = [24, 11]

def setup_ultrasound():
    GPIO.setmode(GPIO.BCM)
    # Sett opp fremre sensorer:
    for trig in trig_pins_front + trig_pins_side:
        GPIO.setup(trig, GPIO.OUT)
        GPIO.output(trig, False)
    for echo in echo_pins_front + echo_pins_side:
        GPIO.setup(echo, GPIO.IN)
    time.sleep(0.5)

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

def check_ultrasound_all():
    """
    Sjekker alle ultralydsensorer og returnerer avstander i en ordbok.
    """
    distances = {
        'front_left': les_avstand(trig_pins_front[0], echo_pins_front[0]),
        'front_right': les_avstand(trig_pins_front[1], echo_pins_front[1]),
        'side_left': les_avstand(trig_pins_side[0], echo_pins_side[0]),
        'side_right': les_avstand(trig_pins_side[1], echo_pins_side[1]),
    }
    return distances

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
    global current_distance_lidar, running, lidar_buffer, lidar_data
    
    for scan in lidar.iter_scans():
        if not running:
            break
            
        # Oppdater lidar_data med nyeste skanning
        current_scan = {int(measurement[1]): measurement[2] for measurement in scan}
        lidar_data.update(current_scan)
        
        # Filtrer ut m�linger i frontretning for hurtig hindringdeteksjon
        front_distances = []
        for angle, distance in current_scan.items():
            # Konverter alle vinkler til omr�det [-180, 180]
            norm_angle = ((angle + 180) % 360) - 180
            
            # Sjekk om vinkelen er innenfor "fremover" omr�de
            if SCAN_ANGLES['front'][0] <= norm_angle <= SCAN_ANGLES['front'][1]:
                front_distances.append(distance)
                
                # Oppbevar data for stabilitet (buffer for hver vinkel)
                if norm_angle not in lidar_buffer:
                    lidar_buffer[norm_angle] = []
                
                lidar_buffer[norm_angle].append(distance)
                if len(lidar_buffer[norm_angle]) > 5:
                    lidar_buffer[norm_angle].pop(0)
        
        # Oppdater current_distance_lidar med minimum avstand foran roboten
        if front_distances:
            current_distance_lidar = min(front_distances)

# ----------------- Kompass-tr�d -----------------
def compass_thread():
    global true_heading, compass_readings, running
    
    while running:
        heading = read_compass()
        if heading >= 0:  # Gyldig avlesning
            compass_readings.append(heading)
            if len(compass_readings) > 10:  # Behold bare de siste 10 m�lingene
                compass_readings.pop(0)
                
            # Beregn medianverdi for � redusere st�y
            true_heading = np.median(compass_readings)
            
        time.sleep(0.05)  # 20 Hz oppdatering

def get_median_distance_in_range(angle_min, angle_max):
    """
    Henter median avstand fra LIDAR-data innenfor et vinkelomr�de.
    Returnerer 9999 hvis ingen data er tilgjengelig.
    """
    distances = []
    
    for angle, distance in lidar_data.items():
        # Normaliser vinkelen til [-180, 180]
        norm_angle = ((angle + 180) % 360) - 180
        if angle_min <= norm_angle <= angle_max:
            distances.append(distance)
    
    if not distances:
        return 9999
    
    return np.median(distances)

def get_median_front_distance():
    """
    Beregner medianverdien av LIDAR-avstand i front-retning.
    """
    return get_median_distance_in_range(SCAN_ANGLES['front'][0], SCAN_ANGLES['front'][1])

def scan_for_best_direction():
    """
    Skanner med LIDAR i alle retninger for � finne den beste �pne veien.
    Returnerer den anbefalte rotasjonsvinkelen (i grader).
    Positiv verdi betyr roter mot venstre, negativ verdi betyr roter mot h�yre.
    """
    distances = {}
    
    # Sjekk avstand i alle definerte retninger
    for direction, (angle_min, angle_max) in SCAN_ANGLES.items():
        median_dist = get_median_distance_in_range(angle_min, angle_max)
        distances[direction] = median_dist
        print(f"Retning {direction}: {median_dist}mm")
    
    # Sjekk f�rst om opprinnelig retning er klar nok
    if distances['front'] > STOP_DISTANCE_LIDAR * 1.5:
        print("Frontruten er klar, fortsetter rett fram")
        return 0
    
    # Sjekk side-retningene og velg den med st�rst avstand
    if distances['left'] > distances['right']:
        if distances['