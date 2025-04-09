import RPi.GPIO as GPIO
import time

# Definerer pinner for de fire HC-SR04-sensorene
# Merk: Her brukes BCM oppsett. Endre nummereringen hvis du bruker BOARD-modus.
# Front Venstre
trigFrontVenstre = 9
echoFrontVenstre = 8

# Front H�yre
trigFrontHoyre = 7
echoFrontHoyre = 6

# Bak Venstre
trigBakVenstre = 23
echoBakVenstre = 24

# Bak H�yre
trigBakHoyre = 10
echoBakHoyre = 11

def les_avstand(trig, echo):
    """
    Sender ut en 10 �s puls og m�ler varigheten p� echo-signalet.
    Returnerer avstanden i cm eller -1 dersom den er utenfor rekkevidde.
    """
    # Sett trig-pinnen lav
    GPIO.output(trig, False)
    time.sleep(0.000002)  # 2 �s
    # Send en 10 �s puls
    GPIO.output(trig, True)
    time.sleep(0.00001)   # 10 �s
    GPIO.output(trig, False)
    
    # M�l tidspunktet n�r echo-pinnen blir h�y
    timeout_start = time.time()
    while GPIO.input(echo) == 0:
        pulse_start = time.time()
        # Timeout etter 30 ms for � unng� evig venting
        if pulse_start - timeout_start > 0.03:
            return -1

    # M�l tidspunktet n�r echo-pinnen g�r lav
    timeout_start = time.time()
    while GPIO.input(echo) == 1:
        pulse_end = time.time()
        # Timeout etter 30 ms
        if pulse_end - timeout_start > 0.03:
            return -1

    # Beregn varigheten i mikrosekunder
    duration = (pulse_end - pulse_start) * 1e6  # konverter til �s
    
    # Hvis varigheten er for lang, anses den som "out range"
    if duration >= 38000:
        return -1
    else:
        # Avstand i cm (lydens hastighet ca. 0,0343 cm/�s, del p� 2 for tur/retur)
        distance = duration / 58.0
        return distance

def setup():
    GPIO.setmode(GPIO.BCM)  # Bruk BCM-nummerering
    # Sett opp alle trig og echo pinner
    sensor_pins = [trigFrontVenstre, trigFrontHoyre, trigBakVenstre, trigBakHoyre]
    echo_pins = [echoFrontVenstre, echoFrontHoyre, echoBakVenstre, echoBakHoyre]
    
    for trig in sensor_pins:
        GPIO.setup(trig, GPIO.OUT)
        GPIO.output(trig, False)
    for echo in echo_pins:
        GPIO.setup(echo, GPIO.IN)
    
    # Vent litt f�r f�rste m�ling
    time.sleep(6)
    print("Avstandsm�ling:")

def loop():
    while True:
        front_venstre = les_avstand(trigFrontVenstre, echoFrontVenstre)
        time.sleep(0.05)
        front_hoyre = les_avstand(trigFrontHoyre, echoFrontHoyre)
        time.sleep(0.05)
        bak_venstre = les_avstand(trigBakVenstre, echoBakVenstre)
        time.sleep(0.05)
        bak_hoyre = les_avstand(trigBakHoyre, echoBakHoyre)
        time.sleep(0.05)
        
        # Skriv ut m�lingene
        if front_venstre < 0:
            print("Front Venstre: Out range")
        else:
            meter = front_venstre / 100.0
            print("Front Venstre: {:.2f} cm  ({:.2f} m)".format(front_venstre, meter))
            
        if front_hoyre < 0:
            print("Front H�yre: Out range")
        else:
            meter = front_hoyre / 100.0
            print("Front H�yre: {:.2f} cm  ({:.2f} m)".format(front_hoyre, meter))
            
        if bak_venstre < 0:
            print("Bak Venstre: Out range")
        else:
            meter = bak_venstre / 100.0
            print("Bak Venstre: {:.2f} cm  ({:.2f} m)".format(bak_venstre, meter))
            
        if bak_hoyre < 0:
            print("Bak H�yre: Out range")
        else:
            meter = bak_hoyre / 100.0
            print("Bak H�yre: {:.2f} cm  ({:.2f} m)".format(bak_hoyre, meter))
            
        print("-----------------------------")
        time.sleep(1)

def destroy():
    GPIO.cleanup()

if __name__ == '__main__':
    try:
        setup()
        loop()
    except KeyboardInterrupt:
        destroy()
