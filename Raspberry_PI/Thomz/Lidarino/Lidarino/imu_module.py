# Filnavn: imu_module.py
# Modul for � h�ndtere IMU-data og rotasjon via gyroskopm�linger
import smbus
import time
import math
from motor_control import send_command  # Importeres her for � unng� sirkul�r avhengighet

# Konfigurasjon
IMU_ADDRESS = 0x68          # Adresse til IMU (sjekk med `i2cdetect -y 1`)
ROTATION_SPEED = 18         # Standard rotasjonshastighet
bus = smbus.SMBus(1)        # Bruker I2C-buss 1

def init_imu():
    """Initialiserer IMU-en (for eksempel MPU6050/9250)."""
    print("Initialiserer IMU...")
    try:
        bus.write_byte_data(IMU_ADDRESS, 0x6B, 0)  # Vekk fra sleep
        time.sleep(0.1)
        print("IMU initialisert.")
    except Exception as e:
        print(f"Feil under initialisering av IMU: {e}")
        raise

def read_word(addr):
    """Leser 16-bit unsigned verdi fra to registre (Big-Endian)."""
    high = bus.read_byte_data(IMU_ADDRESS, addr)
    low = bus.read_byte_data(IMU_ADDRESS, addr + 1)
    return (high << 8) + low

def read_word_2c(addr):
    """Konverterer unsigned 16-bit verdi til signed (to-komplement)."""
    val = read_word(addr)
    return -((65535 - val) + 1) if val >= 0x8000 else val

def get_gyro_z():
    """Returnerer gyroskop Z-akse i grader per sekund."""
    try:
        raw_gyro_z = read_word_2c(0x47)
        return raw_gyro_z / 131.0  # Sensitivitet for �250 deg/s
    except Exception as e:
        print(f"Feil under lesing av gyro Z: {e}")
        return 0.0

def get_accel_xyz_raw():
    """Returnerer r� akselerometerverdier for X, Y, Z."""
    try:
        x = read_word_2c(0x3B)
        y = read_word_2c(0x3D)
        z = read_word_2c(0x3F)
        return x, y, z
    except Exception as e:
        print(f"Feil under lesing av akselerometer: {e}")
        return 0, 0, 0

def rotate_by_gyro(target_angle):
    """
    Roterer roboten til en spesifikk vinkel ved � integrere gyro Z-data.
    Positiv vinkel = �n retning, negativ = motsatt.
    """
    integrated_angle = 0.0
    last_time = time.time()
    last_update = last_time
    update_interval = 0.1

    print(f"Starter rotasjon: {target_angle:.2f}�")
    rotation_speed = ROTATION_SPEED if target_angle > 0 else -ROTATION_SPEED
    send_command(f"0 0 {rotation_speed}\n")

    while abs(integrated_angle) < abs(target_angle):
        current_time = time.time()
        dt = min(current_time - last_time, 0.01)
        last_time = current_time

        # Gyro Z m�ling og integrasjon
        raw_gyro = get_gyro_z()
        compensated = -raw_gyro  # Kompenser for eventuell opp-ned montering
        integrated_angle += compensated * dt

        # Sakke ned ved n�r m�l
        remaining = abs(target_angle) - abs(integrated_angle)
        if remaining < 10:
            slow = max(1, ROTATION_SPEED // 2)
            send_command(f"0 0 {slow if target_angle > 0 else -slow}\n")

        if current_time - last_update >= update_interval:
            print(f"Integ: {integrated_angle:.2f}� / M�l: {target_angle:.2f}� (Gjenst�r: {remaining:.2f}�)")
            last_update = current_time

        time.sleep(0.005)

    send_command("0 0 0\n")
    print(f"Rotasjon fullf�rt ({integrated_angle:.2f}� av {target_angle:.2f}�).")
