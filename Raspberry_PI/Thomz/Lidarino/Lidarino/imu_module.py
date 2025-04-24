# Filnavn: imu_module.py

import smbus
import time
import math
# M� importere send_command her for � unng� sirkul�r import ved modul-load
from motor_control import send_command


bus = smbus.SMBus(1)
IMU_ADDRESS = 0x68 # Sjekk at dette er riktig for din IMU med i2cdetect -y 1
ROTATION_SPEED = 35 # Juster rotasjonshastigheten her

def init_imu():
    """Initialiserer IMU-sensoren (f.eks. MPU6050/MPU9250)."""
    print("Initialiserer IMU...")
    try:
        # Vanlig register 0x6B er Power Management 1, sett til 0 for � vekke fra sleep
        bus.write_byte_data(IMU_ADDRESS, 0x6B, 0)
        time.sleep(0.1)
        print("IMU initialisert vellykket.")
    except Exception as e:
        print(f"Feil under initialisering av IMU p� adresse {IMU_ADDRESS}: {e}")
        raise # Kast feilen videre s� main.py fanger den


def read_word(adr):
    """Leser en 16-bits unsigned verdi fra to 8-bits registre (High byte f�rst)."""
    # De fleste MPU-sensorer er Big-Endian for registerdata
    high = bus.read_byte_data(IMU_ADDRESS, adr)
    low = bus.read_byte_data(IMU_ADDRESS, adr + 1)
    return (high << 8) + low

def read_word_2c(adr):
    """Konverterer en 16-bits unsigned verdi til signed (to-komplement)."""
    val = read_word(adr)
    # Konverterer fra unsigned (0 til 65535) til signed (-32768 til 32767)
    return -((65535 - val) + 1) if val >= 0x8000 else val

def get_gyro_z():
    """Leser r�data fra gyroens Z-akse og konverterer til grader per sekund."""
    # Gyro Z raw data register starter vanligvis p� 0x47
    # Deler p� sensitivitetsskalaen (131.0 for � f� grader/s ved �nsket scale setting, f.eks. +/- 250 deg/s)
    # Sjekk databladet for din IMU og dine registerinnstillinger (hvis du endret dem fra default 0x6B=0)
    try:
        # Merk: Leser fra 0x47 (Gyroskop Z high byte), 0x48 er low byte
        # Dette samsvarer med MPU6050/9250
        # Den r�e gyro-verdien er en signed 16-bit verdi
        raw_gyro_z = read_word_2c(0x47)
        # Konverter til grader per sekund
        # 131.0 er for +/- 250 deg/s range. Hvis du konfigurerte annen range (f.eks. +/- 500 deg/s), m� tallet endres
        sensitivity_scale = 131.0 # Standard for +/- 250 deg/s
        gyro_z_dps = raw_gyro_z / sensitivity_scale

        # P� grunn av opp-ned montering, kan Forteget p� gyro Z v�re snudd.
        # Vi kompenserer for dette direkte i rotate_by_gyro integrasjonslinjen.
        # Hvis du vil snu fortegnet her i stedet, legg til et minus: return -gyro_z_dps
        return gyro_z_dps

    except Exception as e:
         print(f"Feil under lesing av Gyro Z: {e}")
         return 0.0 # Returner 0 ved feil for � unng� krash

def get_accel_xyz_raw():
    """Leser r�data fra akselerometerets X, Y, Z akser."""
    # Akselerometer raw data register starter vanligvis p� 0x3B for X_H
    # Rekkef�lge: X_H, X_L, Y_H, Y_L, Z_H, Z_L (Big-Endian for MPU6050/9250)
    # Dataene er signed 16-bit verdier
    try:
        accel_x = read_word_2c(0x3B) # Akselerometer X
        accel_y = read_word_2c(0x3D) # Akselerometer Y
        accel_z = read_word_2c(0x3F) # Akselerometer Z
        # Merk: Disse r�dataene kan trenge skalering til g (f.eks. dele p� 16384.0 for +/- 2g range)
        # og remapping/fortegnjustering basert p� fysisk montering.
        return accel_x, accel_y, accel_z
    except Exception as e:
        print(f"Feil under lesing av Akselerometer r�data: {e}")
        return 0, 0, 0 # Returner 0 ved feil


def rotate_by_gyro(target_angle):
    """
    Roterer roboten en spesifisert vinkel ved bruk av gyro Z-data.
    Vinkelen er i grader. Positiv = �n retning, Negativ = motsatt retning.
    """
    integrated_angle = 0.0 # Akkumulert rotasjonsvinkel
    last_time = time.time() # Tidspunkt for forrige m�ling
    update_interval = 0.1 # Hvor ofte vi skal skrive ut status (sekunder)
    last_update = time.time() # Tidspunkt for forrige status-utskrift

    print(f"Starter rotasjon med m�lvinkel: {target_angle:.2f}�")

    # Bestem rotasjonshastighet og retning basert p� m�lvinkelen
    rotation_speed_signed = ROTATION_SPEED if target_angle > 0 else -ROTATION_SPEED

    # Send startkommando for rotasjon
    send_command(f"0 0 {rotation_speed_signed}\n") # <-- N� bruker vi den signed_speed!

    # Roter til den akkumulerte vinkelen (absoluttverdi) n�r absoluttverdien av m�lvinkelen
    while abs(integrated_angle) < abs(target_angle):
        current_time = time.time()
        dt = current_time - last_time # Tidsforskjell siden forrige m�ling
        # Forhindre for store dt hvis tidsm�ling hopper (f.eks. ved debugging pause)
        if dt > 0.1: dt = 0.01 # Sett en maksgrense p� dt
        last_time = current_time # Oppdater tidspunkt for neste m�ling

        # Les gyro Z og integrer.
        # Vi kompenserer for Forteget p� gyro Z data her pga opp-ned montering.
        # Hvis get_gyro_z() gir motsatt fortegn av den faktiske rotasjonen,
        # m� vi snu fortegnet p� data f�r integrering.
        raw_gyro_dps = get_gyro_z() # Hent den skalerte gyroverdien
        # Kompenser for Forteget. Hvis Z-aksen er snudd, snu Forteget p� m�lingen.
        # Antar her at opp-ned kun snur Forteget p� Z. Hvis det snur retning OG Forteget,
        # m� logikken v�re mer kompleks. Men start med � snu Forteget:
        compensated_gyro_dps = -raw_gyro_dps # <-- Legger til MINUS her for � snu Forteget!

        # N� integrer den Fortegets-kompenserte gyrodataen.
        # Skaleringen (1 if rotation_speed_signed > 0 else -1) er kun for � matche
        # Forteget p� den akkumulerte vinkelen til Forteget p� m�lvinkelen.
        integrated_angle += compensated_gyro_dps * dt # Integrer. Denne skal n� akkumuleres i riktig retning basert p� Forteget p� compensated_gyro_dps.

        # VIKTIG: Test om Forteget p� 'compensated_gyro_dps' stemmer overens
        # med Forteget p� 'rotation_speed_signed'. Hvis rotation_speed_signed > 0
        # og roboten snur, b�r compensated_gyro_dps n� v�re positiv. Hvis ikke,
        # m� kanskje Forteget snus igjen i compensated_gyro_dps = ... linjen.
        # Alternativt kan integrasjonslinjen v�re:
        # integrated_angle += abs(compensated_gyro_dps) * dt * (1 if rotation_speed_signed > 0 else -1)
        # Hvis Forteget p� compensated_gyro_dps ER til � stole p�, kan du bare bruke:
        # integrated_angle += compensated_gyro_dps * dt
        # Men la oss starte med � kompensere Forteget f�r integrering.


        # Saks ned n�r vi n�rmer oss m�lvinkelen
        remaining_angle = abs(target_angle) - abs(integrated_angle)
        if remaining_angle < 10: # Hvis mindre enn 10 grader igjen (juster terskelen)
             slow_speed = max(1, ROTATION_SPEED // 2) # Bremse til halv fart (minst 1)
             # Saks ned i riktig retning
             slow_speed_signed = slow_speed if target_angle > 0 else -slow_speed
             send_command(f"0 0 {slow_speed_signed}\n") # <-- N� bruker vi den signed_speed!
             # print("Sakker ned...") # Debugging

        # Skriv ut status med jevne mellomrom
        if current_time - last_update >= update_interval:
            # print(f"R� gyro: {raw_gyro_dps:.2f} compensated: {compensated_gyro_dps:.2f} integ: {integrated_angle:.2f}� / M�l: {target_angle:.2f}� (Gjenst�ende: {remaining_angle:.2f}�)")
            print(f"Integ: {integrated_angle:.2f}� / M�l: {target_angle:.2f}� (Gjenst�ende: {remaining_angle:.2f}�)")
            last_update = current_time

        # Liten pause for � ikke overbelaste loopen
        time.sleep(0.005)

    # Stopp rotasjonen n�r m�let er n�dd
    send_command("0 0 0\n")
    print(f"Rotasjon fullf�rt n�r m�let ({integrated_angle:.2f}� av {target_angle:.2f}�).")
    # Det kan v�re lurt � gj�re en liten korreksjon her hvis vinkelen ble over- eller underskutt


# --- Hjelpefunksjon for cleanup (valgfritt) ---
# Hvis IMU-en trenger spesiell cleanup ved avslutning, legg det til her.
# def cleanup_imu():
#    pass # Forel�pig ingen cleanup n�dvendig for denne IMU-en


# Ingen if __name__ == "__main__": blokk her, da main.py starter alt.