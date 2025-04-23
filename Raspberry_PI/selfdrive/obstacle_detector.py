import RPi.GPIO as GPIO
import time
import threading

TRIG = 23
ECHO = 24
DIST_THRESHOLD = 30  # i cm
callback = None
running = True

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)

def read_distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start = time.time()
    while GPIO.input(ECHO) == 0:
        start = time.time()
    while GPIO.input(ECHO) == 1:
        end = time.time()

    duration = end - start
    distance = duration * 34300 / 2
    return distance

def monitor_distance():
    global callback, running
    try:
        while running:
            dist = read_distance()
            print(f"[Hinderdeteksjon] Avstand: {dist:.1f} cm")
            if dist < DIST_THRESHOLD and callback:
                callback()
            time.sleep(0.2)
    except:
        GPIO.cleanup()

def start_monitoring(cb):
    global callback
    callback = cb
    setup()
    thread = threading.Thread(target=monitor_distance, daemon=True)
    thread.start()

def stop_monitoring():
    global running
    running = False
    GPIO.cleanup()
