# Filnavn: compass_module.py

import smbus
import time
import math

# I2C-adresse for QMC5883L
QMC5883L_ADDRESS = 0x0d # Standard adresse
bus = smbus.SMBus(1) # Standard I2C-buss p� Raspberry Pi (oftest 1)

# Registeradresser for QMC5883L
QMC5883L_CTRL1 = 0x09
QMC5883L_SET_RESET = 0x0B
QMC5883L_DATA = 0x00 # Startregister for X_L data

def init_compass():
    """Initialiserer QMC5883L med riktige innstillinger."""
    # Skriv til Reset-registeret (0x0B) for � utf�re en soft reset
    try:
        bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_SET_RESET, 0x01)
        time.sleep(0.1) # Gi sensoren litt tid etter reset

        # Skriv til kontrollregister 1 (0x09) for � sette modus, data rate, range, oversampling
        # 0b00011101 tilsvarer typisk:
        # Bit 7-6: Mode (01 = Continuous)
        # Bit 5-4: ODR (Output Data Rate) (01 = 50Hz)
        # Bit 3-2: RNG (Range) (11 = 8G)
        # Bit 1-0: OSR (Over Sampling Ratio) (01 = 256x)
        bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_CTRL1, 0b00011101) # Kontinuerlig modus
        time.sleep(0.1) # Gi sensoren litt tid til � starte m�linger
        print("QMC5883L initialisert vellykket.")
    except Exception as e:
        print(f"Feil under initialisering av QMC5883L: {e}")
        # Vurder � h�ndtere feilen, f.eks. avslutte eller pr�ve p� nytt

def read_compass():
    """
    Leser r�data fra QMC5883L (X, Y, Z) og beregner heading i grader.
    Returnerer heading i grader og r�data (X, Y, Z).
    """
    try:
        # Les 6 byte data fra QMC5883L fra register 0x00
        # Rekkef�lgen er X_L, X_H, Y_L, Y_H, Z_L, Z_H (Little-Endian)
        data = bus.read_i2c_block_data(QMC5883L_ADDRESS, QMC5883L_DATA, 6)

        # Rekonstruer 16-bits verdiene
        # QMC5883L er Little-Endian: (High << 8) | Low
        x_raw = (data[1] << 8) | data[0]
        y_raw = (data[3] << 8) | data[2]
        z_raw = (data[5] << 8) | data[4]

        # Konverter fra usigned 16-bit til signed (to-komplement)
        x = x_raw - 65536 if x_raw > 32767 else x_raw
        y = y_raw - 65536 if y_raw > 32767 else y_raw
        z = z_raw - 65536 if z_raw > 32767 else z_raw # Z-aksen trengs ofte ikke for 2D heading, men leses uansett

        # Beregn heading i radianer ved hjelp av atan2(y, x)
        # atan2 tar hensyn til alle kvadranter
        heading_rad = math.atan2(y, x) # atan2(y, x) i henhold til datablad og standard praksis

        # Konverter fra radianer til grader
        heading_deg = math.degrees(heading_rad)

        # Normaliser heading til � v�re mellom 0 og 360 grader
        # atan2 returnerer mellom -180 og +180. Vi vil ha 0-360.
        if heading_deg < 0:
            heading_deg += 360

        # Mer robust normalisering som alltid gir [0, 360)
        # heading_deg = (heading_deg + 360) % 360

        # main.py forventer kun heading, men returnerer ogs� raw data hvis de trengs
        return heading_deg #main.py bruker kun den f�rste verdien
    except Exception as e:
        print(f"Feil under lesing av QMC5883L: {e}")
        # Returner en verdi som indikerer feil, f.eks. -1 eller None
        return -1 # Returnerer -1 for heading ved feil


def get_compass_direction(deg):
    """Konverterer grader til en kardinal retning (N, NE, E, osv.)."""
    # Liste over de 16 hovedretningene for litt mer n�yaktighet
    dirs = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW']
    # Beregn indeks basert p� gradene. Legg til 11.25 (halvparten av 22.5) og modulus 360
    # for � sentrere retningene rundt nord (0/360 grader). Del p� 22.5 for � f� indeksen.
    # Vi har 16 retninger, s� 360 / 16 = 22.5 grader per sektor.
    if deg < 0 or deg > 360: # H�ndter eventuelle unormale inndata
         return "Ukjent"
    return dirs[int((deg + 11.25) % 360 // 22.5)]

# Ingen if __name__ == "__main__": blokk her, da main.py starter alt.