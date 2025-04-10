import RPi.GPIO as GPIO
import time

# Definer trig- og echo-pinner for sensorene
# Rekkef�lge: front venstre, front h�yre, bak venstre, bak h�yre
trig_pins = [9, 7, 23, 10]
echo_pins = [8, 6, 24, 11]
sensor_names = ["front_left", "front_right", "back_left", "back_right"]

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
    """
    Leser avstand fra en ultralydsensor.
    Sender ut en kort puls og m�ler tiden det tar f�r echo-signalet kommer tilbake.
    Avstanden beregnes i centimeter.
    """
    GPIO.output(trig, False)
    time.sleep(0.000002)
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    
    timeout_start = time.time()
    # Vent p� at echo g�r HIGH (puls starter)
    while GPIO.input(echo) == 0:
        pulse_start = time.time()
        if pulse_start - timeout_start > 0.03:  # timeout p� 30 ms
            return -1

    # Merk starten p� signalet
    while GPIO.input(echo) == 1:
        pulse_end = time.time()
        if pulse_end - pulse_start > 0.03:
            return -1

    pulse_duration = (pulse_end - pulse_start) * 1e6  # konverter til mikrosekunder

    # Konverter varigheten til avstand i cm
    distance_cm = pulse_duration / 58.0
    return distance_cm

if __name__ == "__main__":
    try:
        setup_ultrasound()
        while True:
            for i, (trig, echo) in enumerate(zip(trig_pins, echo_pins)):
                d = read_distance(trig, echo)
                # Skriv ut m�lingen; hvis d er -1 vises "feil m�ling"
                if d == -1:
                    print(f"{sensor_names[i]}: Feil m�ling")
                else:
                    print(f"{sensor_names[i]}: {d:.2f} cm")
            print("-" * 40)
            time.sleep(1)  # pause p� 1 sekund mellom hver runde
    except KeyboardInterrupt:
        print("Avslutter testprogrammet.")
    finally:
        GPIO.cleanup()
