import logging
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

# InfluxDB-innstillinger (oppdater med deres verdier)
INFLUX_URL = "https://eu-central-1-1.aws.cloud2.influxdata.com/"
INFLUX_TOKEN = "ZbCkxFHS5X0_9PG3YMeznY07WFCsMfaV7MMTSWyb7Ckq72zxMdVq2rB20ZdyxccfB1di9wQOBguZv70A9VEErA=="
INFLUX_ORG = "61778c39081df8c1"
INFLUX_BUCKET = "Lidarino"

client = InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG)
write_api = client.write_api(write_options=SYNCHRONOUS)

def log_debug_data(data):
    try:
        point = Point("debug_data")
        for key, value in data.items():
            if value is not None:
                point = point.field(key, value)
        write_api.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=point)
        logging.info("Debugdata sendt til InfluxDB.")
    except Exception as e:
        logging.error(f"Feil ved sending til InfluxDB: {e}")

def close_client():
    client.close()
