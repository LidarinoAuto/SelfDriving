# Filnavn: ultrasound_module.py
import RPi.GPIO as GPIO
import time

# --- Konfigurasjon av GPIO-pinner (BCM-nummerering) ---

# Frontsensorer
trig_pins_front = [9, 7]
echo_pins_front = [8, 6]

# Baksensorer
trig_pins_back = [23, 10]
echo_pins_back = [24, 11]

# Samle alle trig- og echo-pinner
all_trig_pins = trig_pins_front + trig_pins_back
all_echo_pins = echo_pins_front + echo_pins_back

# Avstandsterskel (i cm) for � registrere hindring
STOP_DISTANCE_ULTRASOUND = 5


def setup_ultrasound():
    """Setter opp GPIO-pinner for alle tilkoblede ultralydsensorer."""
    print("Setter opp ultralydsensorer...")
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    for trig in all_trig_pins:
        GPIO.setup(trig, GPIO.OUT)
        GPIO.output(trig, False)

    for echo in all_echo_pins:
        GPIO.setup(echo, GPIO.IN)

    time.sleep(2)  # La sensorene stabilisere seg
    print("Ultralydsensorer satt opp.")


def les_avstand(trig, echo):
    """
    Sender ut puls og m�ler retur for �n HC-SR04 sensor.
    Returnerer avstanden i cm, eller -1 ved feil eller timeout.
    """
    GPIO.output(trig, False)
    time.sleep(0.000002)
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    timeout_start = time.time()
    pulse_start = time.time()
    while GPIO.input(echo) == 0:
        pulse_start = time.time()
        if time.time() - timeout_start > 0.03:
            return -1

    timeout_start = time.time()
    pulse_end = time.time()
    while GPIO.input(echo) == 1:
        pulse_end = time.time()
        if time.time() - timeout_start > 0.03:
            return -1

    duration_us = (pulse_end - pulse_start) * 1e6

    if duration_us <= 0 or duration_us >= 38000:
        return -1

    return duration_us / 58.0  # Konverter til cm


def check_ultrasound_all():
    """
    Leser alle ultralydsensorer sekvensielt.
    Returnerer liste over indeksene til sensorene som detekterer hindringer.
    """
    triggered_sensors = []

    for i in range(len(all_trig_pins)):
        d = les_avstand(all_trig_pins[i], all_echo_pins[i])

        if d > 0 and d < STOP_DISTANCE_ULTRASOUND:
            triggered_sensors.append(i)
            print(f"Ultralydsensor {i} ({all_trig_pins[i]}/{all_echo_pins[i]}) hindring: {d:.2f} cm!")

        time.sleep(0.05)  # Reduser risiko for kryssinterferens

    return triggered_sensors


def cleanup_ultrasound():
    """Rydder opp GPIO-pinner brukt av ultralydsensorene."""
    print("Rydder opp GPIO for ultralyd...")
    try:
        GPIO.cleanup(all_trig_pins + all_echo_pins)
        print("GPIO opprydding fullf�rt.")
    except Exception as e:
        print(f"Feil under GPIO-opprydding: {e}")
