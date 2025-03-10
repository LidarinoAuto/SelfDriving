import RPi.GPIO as GPIO
import time
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

# ? InfluxDB-innstillinger (FYLL INN DINE VERDIER)
INFLUX_URL = "https://eu-central-1-1.aws.cloud2.influxdata.com/"  # Endre region hvis n�dvendig
INFLUX_TOKEN = "ZbCkxFHS5X0_9PG3YMeznY07WFCsMfaV7MMTSWyb7Ckq72zxMdVq2rB20ZdyxccfB1di9wQOBguZv70A9VEErA=="
INFLUX_ORG = "61778c39081df8c1"
INFLUX_BUCKET = "Lidarino"

# ? Opprett InfluxDB-klient
client = InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG)
write_api = client.write_api(write_options=SYNCHRONOUS)

# ? Definerer pinner for de fire HC-SR04-sensorene (BCM-modus)
SENSORS = {
    "FrontVenstre": {"trig": 9, "echo": 8},
    "FrontHoyre": {"trig": 7, "echo": 6},
    "BakVenstre": {"trig": 23, "echo": 24},
    "BakHoyre": {"trig": 10, "echo": 11}
}

# ? Funksjon for � lese avstand fra en sensor
def les_avstand(trig, echo):
    GPIO.output(trig, False)
    time.sleep(0.000002)  # Kort pause
    GPIO.output(trig, True)
    time.sleep(0.00001)  # 10 �s puls
    GPIO.output(trig, False)

    timeout_start = time.time()
    while GPIO.input(echo) == 0:
        pulse_start = time.time()
        if pulse_start - timeout_start > 0.03:
            return -1  # Timeout

    timeout_start = time.time()
    while GPIO.input(echo) == 1:
        pulse_end = time.time()
        if pulse_end - timeout_start > 0.03:
            return -1  # Timeout

    # Beregn avstand (lydens hastighet: 343 m/s eller 0.0343 cm/�s)
    duration = (pulse_end - pulse_start) * 1e6  # �s
    distance = duration / 58.0  # cm
    return -1 if duration >= 38000 else distance

# ? Oppsett av GPIO
def setup():
    GPIO.setmode(GPIO.BCM)
    for sensor in SENSORS.values():
        GPIO.setup(sensor["trig"], GPIO.OUT)
        GPIO.output(sensor["trig"], False)
        GPIO.setup(sensor["echo"], GPIO.IN)
    time.sleep(1)  # Vent f�r f�rste m�ling

# ? Hovedl�kke: M�ler og sender data til InfluxDB
def loop():
    while True:
        data = {}
        for navn, pinner in SENSORS.items():
            distanse = les_avstand(pinner["trig"], pinner["echo"])
            data[navn] = distanse if distanse >= 0 else None  # Lagre -1 som None

        # ? Skriv ut m�linger
        print("? Avstandsm�linger:")
        for navn, avstand in data.items():
            print(f"{navn}: {avstand:.2f} cm" if avstand else f"{navn}: Utenfor rekkevidde")

        print("-" * 40)

        # ? Send til InfluxDB
        try:
            point = Point("ultrasonic_distance")
            for navn, avstand in data.items():
                if avstand is not None:
                    point = point.field(navn, avstand)  # Legg til felt for hver sensor

            write_api.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=point)
            print("? Data sendt til InfluxDB!")
        except Exception as e:
            print(f"? Feil ved sending: {e}")

        time.sleep(1)  # Vent 1 sekund mellom m�linger

# ? Rydd opp ved avbrudd
def destroy():
    GPIO.cleanup()
    client.close()
    print("? Rydder opp og avslutter.")

# ? Start programmet
if __name__ == '__main__':
    try:
        setup()
        loop()
    except KeyboardInterrupt:
        destroy()
