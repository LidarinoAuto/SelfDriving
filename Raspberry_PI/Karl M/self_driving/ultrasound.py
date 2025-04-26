# ultrasound.py
import RPi.GPIO as GPIO
import time

# Trig og Echo pinner: front venstre, front høyre, bak venstre, bak høyre
trig_pins = [9, 7, 23, 10]
echo_pins = [8, 6, 24, 11]
sensor_names = ["front_left", "front_right", "back_left", "back_right"]

# Vinkler i grader relativt til rett frem (0°)
sensor_angles = {
    "front_left": 340,
    "front_right": 20,
    "back_left": 222,
    "back_right": 138
}

# Målinger lagres her
sensor_distances = {
    "front_left": -1,
    "front_right": -1,
    "back_left": -1,
    "back_right": -1
}

# Samle alle trig/echo-par i en dict
sensors = {
    "front_left": (trig_pins[0], echo_pins[0]),
    "front_right": (trig_pins[1], echo_pins[1]),
    "back_left": (trig_pins[2], echo_pins[2]),
    "back_right": (trig_pins[3], echo_pins[3]),
}

def setup_ultrasound():
    GPIO.setmode(GPIO.BCM)
    for trig in trig_pins:
        GPIO.setup(trig, GPIO.OUT)
        GPIO.output(trig, False)
    for echo in echo_pins:
        GPIO.setup(echo, GPIO.IN)
    print("Ultralydsensorene er satt opp. Vent 2 sekunder for stabilisering...")
    time.sleep(2)

def read_distance(trig, echo):
    # Send kort triggerpuls
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    timeout_start = time.time()
    timeout = 0.02  # maks 20 ms venting på start av ekko

    # Vent på ekko start (echo blir HIGH)
    while GPIO.input(echo) == 0:
        pulse_start = time.time()
        if pulse_start - timeout_start > timeout:
            return -1  # Timeout

    # Vent på ekko slutt (echo blir LOW)
    timeout_start = time.time()
    while GPIO.input(echo) == 1:
        pulse_end = time.time()
        if pulse_end - timeout_start > timeout:
            return -1  # Timeout

    pulse_duration = (pulse_end - pulse_start) * 1e6  # mikrosekunder
    distance = pulse_duration * 0.0343 / 2  # lydhastighet, tur/retur
    return distance

def update_ultrasound_readings():
    for sensor, (trig, echo) in sensors.items():
        distance = read_distance(trig, echo)
        if distance > 0:
            sensor_distances[sensor] = distance
        else:
            sensor_distances[sensor] = 0  # Eller -1 hvis du heller vil ignorere det
