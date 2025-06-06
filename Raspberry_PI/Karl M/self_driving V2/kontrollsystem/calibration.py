# calibration.py
import time
from sensorer import mpu6050
from sensorer import kompas
from kontrollsystem import motorsignal
import pygame

def logg(melding):
    """Skriver en melding med tidstempel til loggfil."""
    with open("kalibreringslogg.txt", "a") as f:
        tidspunkt = time.strftime("%Y-%m-%d %H:%M:%S")
        f.write(f"[{tidspunkt}] {melding}\n")

gyro_offset = 0
compass_offset_x = 0
compass_offset_y = 0

def calibrate_gyro():
    print("Starter gyro-kalibrering...")
    logg("Starter gyro-kalibrering")
    mpu6050.setup_mpu6050()

    samples = 1000
    total = 0

    for _ in range(samples):
        gz = mpu6050.read_gyro_z_raw()
        total += gz
        time.sleep(0.005)

    global gyro_offset
    gyro_offset = total / samples
    print(f"Gyro offset målt: {gyro_offset:.5f} grader/s")
    logg(f"Gyro offset målt: {gyro_offset:.5f} grader/s")

    with open("gyro_offset.txt", "w") as f:
        f.write(f"{gyro_offset}\n")
    logg("Gyro-offset lagret til gyro_offset.txt.")

def calibrate_compass():
    print("Starter kompass-kalibrering...")
    logg("Starter kompass-kalibrering")
    kompas.setup_compass()

    rotation_speed = 30  # grader per sekund
    rotation_duration = 15  # sekunder (mer enn 360� rotasjon)

    min_x = min_y = 32767
    max_x = max_y = -32768

    logg("Starter rotasjon mot venstre")
    motorsignal.send_movement_command(0, 0, rotation_speed)

    start_time = time.time()

    while (time.time() - start_time) < rotation_duration:
        x, y = kompas.read_raw_xy()
        min_x = min(min_x, x)
        max_x = max(max_x, x)
        min_y = min(min_y, y)
        max_y = max(max_y, y)

        logg(f"Lest kompass X={x}, Y={y}")
        pygame.event.pump()
        time.sleep(0.01)

    motorsignal.send_movement_command(0, 0, 0.0)
    logg("Stoppet rotasjon")
    time.sleep(1)

    # --- Regn ut offset ---
    compass_offset_x = (max_x + min_x) / 2
    compass_offset_y = (max_y + min_y) / 2

    print(f"Kompass offset m�lt: x={compass_offset_x:.2f}, y={compass_offset_y:.2f}")
    logg(f"Kompass offset m�lt: x={compass_offset_x:.2f}, y={compass_offset_y:.2f}")

    # --- Lagre til fil ---
    with open("kompas_offset.txt", "w") as f:
        f.write(f"{compass_offset_x}\n")
        f.write(f"{compass_offset_y}\n")
    
    logg("Kompass-offset lagret til kompas_offset.txt.")

def full_calibration():
    logg("Starter full kalibrering")
    calibrate_gyro()
    calibrate_compass()
    logg("Full kalibrering ferdig")
    print("Full kalibrering ferdig!")
