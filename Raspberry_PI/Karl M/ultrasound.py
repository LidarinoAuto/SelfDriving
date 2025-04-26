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

    while GPIO.input(echo) == 1:
        pulse_end = time.time()
        if pulse_end - timeout_start > 0.03:
            return -1

    pulse_duration = (pulse_end - pulse_start) * 1e6
    distance_cm = pulse_duration / 58.0
    return distance_cm

def update_ultrasound_readings():
    for i, (trig, echo) in enumerate(zip(trig_pins, echo_pins)):
        name = sensor_names[i]
        distance = read_distance(trig, echo)
        sensor_distances[name] = distance
