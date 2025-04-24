# Filnavn: compass_module.py
import smbus
import time
import math

# --- I2C-adresse og registeradresser for QMC5883L --- 
QMC5883L_ADDRESS = 0x0d  # Standard I2C-adresse for QMC5883L
bus = smbus.SMBus(1)      # Standard I2C-buss p� Raspberry Pi (ofte 1)

# Registeradresser for QMC5883L
QMC5883L_CTRL1 = 0x09        # Kontrollregister 1 for sensorinnstillinger
QMC5883L_SET_RESET = 0x0B    # Reset-register
QMC5883L_DATA = 0x00         # Startregister for X_L data (data begynner her)

def init_compass():
    """Initialiserer QMC5883L med riktige innstillinger."""
    try:
        # Utf�r en soft reset ved � skrive til reset-registeret
        bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_SET_RESET, 0x01)
        time.sleep(0.1)  # Gi sensoren litt tid etter reset

        # Skriv til kontrollregister 1 for � sette modus og innstillinger
        # Kontinuerlig modus, 50Hz ODR, 8G range, 256x oversampling
        bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_CTRL1, 0b00011101)
        time.sleep(0.1)  # Vent p� at sensoren starter m�linger

        print("QMC5883L initialisert vellykket.")
    except Exception as e:
        print(f"Feil under initialisering av QMC5883L: {e}")
        # Vurder � h�ndtere feilen videre, for eksempel avslutte eller pr�ve p� nytt

def read_compass():
    """
    Leser r�data fra QMC5883L (X, Y, Z) og beregner heading i grader.
    Returnerer heading i grader og r�data (X, Y, Z).
    """
    try:
        # Les 6 byte med data fra QMC5883L, starter fra register 0x00
        # Dataene kommer i rekkef�lgen X_L, X_H, Y_L, Y_H, Z_L, Z_H (Little-Endian)
        data = bus.read_i2c_block_data(QMC5883L_ADDRESS, QMC5883L_DATA, 6)

        # Rekonstruer 16-bits verdier fra de 2 byte (Little-Endian)
        x_raw = (data[1] << 8) | data[0]
        y_raw = (data[3] << 8) | data[2]
        z_raw = (data[5] << 8) | data[4]

        # Konverter fra usignerte 16-bits verdier til signed (to-komplement)
        x = x_raw - 65536 if x_raw > 32767 else x_raw
        y = y_raw - 65536 if y_raw > 32767 else y_raw
        z = z_raw - 65536 if z_raw > 32767 else z_raw

        # Beregn heading i radianer ved hjelp av atan2(y, x) for � ta hensyn til alle kvadranter
        heading_rad = math.atan2(y, x)

        # Konverter fra radianer til grader
        heading_deg = math.degrees(heading_rad)

        # Normaliser heading til � v�re mellom 0 og 360 grader
        if heading_deg < 0:
            heading_deg += 360

        return heading_deg  # Returner kun headingen som brukes i main.py

    except Exception as e:
        print(f"Feil under lesing av QMC5883L: {e}")
        # Returner -1 ved feil (kan h�ndteres videre i hovedprogrammet)
        return -1

def get_compass_direction(deg):
    """
    Konverterer grader til en kardinal retning (N, NE, E, osv.).
    Returnerer en retning basert p� input grader.
    """
    # Liste over de 16 hovedretningene for presis navigering
    dirs = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW']

    # Beregn indeks basert p� grader. Legg til 11.25 for � sentrere rundt nord (0/360 grader)
    # 360 / 16 = 22.5 grader per sektor, som gir 16 kardinalpunkter
    if deg < 0 or deg > 360:  # H�ndter eventuelle unormale grader
        return "Ukjent"

    # Returner den beregnede retningen
    return dirs[int((deg + 11.25) % 360 // 22.5)]

# Merk: Ingen if __name__ == "__main__": blokk her. Koden starter fra main.py.
