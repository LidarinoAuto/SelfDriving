# Filnavn: ultrasound_module.py

import RPi.GPIO as GPIO
import time

# Definerer pinner for de fire HC-SR04-sensorene (BCM-nummerering)
# Front Venstre
trig_pins_front = [9]
echo_pins_front = [8]

# Front H�yre
trig_pins_front.append(7)
echo_pins_front.append(6)

# Bak Venstre
trig_pins_back = [23]
echo_pins_back = [24]

# Bak H�yre
trig_pins_back.append(10)
echo_pins_back.append(11)

# Samler alle pinner i lister
all_trig_pins = trig_pins_front + trig_pins_back
all_echo_pins = echo_pins_front + echo_pins_back

STOP_DISTANCE_ULTRASOUND = 5 # Avstand i cm for � trigge stopp

def setup_ultrasound():
    """Konfigurerer GPIO-pinner for ultralydsensorene."""
    print("Setter opp ultralydsensorer...")
    # Sett GPIO-modus til BCM
    GPIO.setmode(GPIO.BCM)
    # Undertrykk advarsler om pinner som allerede er i bruk
    GPIO.setwarnings(False)

    # Sett opp alle trig-pinner som OUTPUT og initialiser lav
    for trig in all_trig_pins:
        GPIO.setup(trig, GPIO.OUT)
        GPIO.output(trig, False)

    # Sett opp alle echo-pinner som INPUT
    for echo in all_echo_pins:
        GPIO.setup(echo, GPIO.IN)

    # Gi sensorene og GPIO litt tid til � stabilisere seg
    time.sleep(2)
    print("Ultralydsensorer satt opp.")


def les_avstand(trig, echo):
    """
    Sender ut en 10 �s puls og m�ler varigheten p� echo-signalet
    for �n sensor. Bruker n�yaktigheten (eller mangelen p� s�dan)
    til time.time().
    Returnerer avstanden i cm eller -1 dersom den er utenfor rekkevidde
    eller en timeout skjer.
    """
    # Sett trig-pinnen lav og vent 2 �s
    GPIO.output(trig, False)
    time.sleep(0.000002)

    # Send en 10 �s h�y puls p� trig-pinnen
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    # M�l tidspunktet n�r echo-pinnen blir h�y (starten p� ekkopulsen)
    # Bruk timeout for � unng� uendelig venting
    pulse_start = time.time() # Initial verdi f�r while-l�kken starter
    timeout_start = time.time()
    while GPIO.input(echo) == 0:
        pulse_start = time.time()
        # Sjekk timeout (f.eks. 30 ms tilsvarer ca. 5 meter)
        if time.time() - timeout_start > 0.03:
             # print(f"Timeout - Puls start aldri h�y for trig={trig}, echo={echo}") # Debugging
             return -1 # Returner feilkode ved timeout

    # M�l tidspunktet n�r echo-pinnen g�r lav (slutten p� ekkopulsen)
    # Bruk timeout for � unng� uendelig venting
    pulse_end = time.time() # Initial verdi f�r while-l�kken starter
    timeout_start = time.time()
    while GPIO.input(echo) == 1:
        pulse_end = time.time()
         # Sjekk timeout
        if time.time() - timeout_start > 0.03:
             # print(f"Timeout - Puls slutt aldri lav for trig={trig}, echo={echo}") # Debugging
             return -1 # Returner feilkode ved timeout

    # Beregn varigheten av pulsen i sekunder, og konverter til mikrosekunder
    duration_seconds = pulse_end - pulse_start
    duration_us = duration_seconds * 1e6 # konverter til �s

    # Hvis varigheten er for lang (f.eks. over 38ms = 6.5 meter), anses den som "out of range"
    # eller en feil m�ling. 38000 �s er ca 6.5 meter.
    if duration_us >= 38000 or duration_us <= 0: # Sjekk ogs� for null eller negativ varighet
        # print(f"Out of range eller feil for trig={trig}, echo={echo}: {duration_us:.2f} �s") # Debugging
        return -1
    else:
        # Beregn avstand i cm. Lydens hastighet er ca 343 m/s = 0.0343 cm/�s.
        # Turen er dobbel avstand (til objektet og tilbake), s� del p� 2.
        # (duration_us * 0.0343) / 2 = duration_us / 58.0
        distance_cm = duration_us / 58.0
        # print(f"M�lt avstand for trig={trig}, echo={echo}: {distance_cm:.2f} cm") # Debugging
        return distance_cm


def check_ultrasound_all():
    """
    Leser avstand fra alle definerte ultralydsensorer sekvensielt.
    Returnerer en LISTE over indekser (0-3) til alle sensorer i listen
    som m�ler avstand under STOP_DISTANCE_ULTRASOUND (15 cm).
    Returnerer en tom liste hvis ingen sensor detekterer en n�r hindring.
    """
    triggered_sensors = [] # Liste for � samle indekser som detekterer

    # G� gjennom alle sensorpar
    for i in range(len(all_trig_pins)):
        trig = all_trig_pins[i]
        echo = all_echo_pins[i]

        # Les avstand fra denne sensoren
        d = les_avstand(trig, echo)

        # Skriv ut m�lingen (valgfritt)
        # if d > 0:
        #    print(f"Sensor {i} ({trig}/{echo}): {d:.2f} cm")
        # else:
        #    print(f"Sensor {i} ({trig}/{echo}): Out range / Feil")

        # Sjekk om avstanden indikerer en n�r hindring
        if d > 0 and d < STOP_DISTANCE_ULTRASOUND:
            # Vi fant en hindring som er n�r nok. Legg til indeksen i listen.
            triggered_sensors.append(i)
            print(f"Ultralydsensor {i} ({trig}/{echo}) hindring: {d:.2f} cm!")

        # Legg til en liten pause mellom avlesningene for � redusere interferens
        time.sleep(0.05) # Denne pausen er viktig!

    # Returner listen over alle triggede sensor-indekser
    return triggered_sensors


def cleanup_ultrasound():
    """Funksjon for � rydde opp GPIO-pinner brukt av ultralydsensorene."""
    print("Rydder opp GPIO for ultralyd...")
    try:
        # Rense kun pinnene vi brukte her
        GPIO.cleanup(all_trig_pins + all_echo_pins)
        print("GPIO opprydding fullf�rt.")
    except Exception as e:
        print(f"Feil under GPIO opprydding: {e}")

# Merk: Ingen if __name__ == "__main__": blokk her. Denne modulen kalles fra main.py.