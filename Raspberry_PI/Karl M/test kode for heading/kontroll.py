# kontroll.py

import kompas
import mpu6050
import calibration
from motorsignal import send_movement_command
from heading import HeadingTracker
import lidar
import ultrasound
import time

# Konstanter
STEP = 100
ROTATE = 15.0
KOMPASS_JUSTERING = 270

# For LIDAR historikk
lidar_points = []
MAX_LIDAR_POINTS = 200

def main():
    # --- KALIBRERING ---
    print('Kalibrerer gyro...')
    calibration.calibrate_gyro()

    print('Kalibrerer kompass...')
    calibration.calibrate_compass()

    print('Kalibrering ferdig!')
    time.sleep(2)

    # --- SENSOROPPSTART ---
    lidar.start_lidar()
    ultrasound.setup_ultrasound()
    kompas.setup_compass()
    mpu6050.setup_mpu6050()
    heading_tracker = HeadingTracker()
    heading_tracker.setup()

    modus = "manuell"
    prev_command = (0, 0, 0.0)

    running = True
    while running:
        # For testing uten pygame må du selv bestemme når du skal avslutte
        # eller sette opp noe som leser input fra f.eks. terminal eller fil
        # Her simulerer vi bare kjøring i manuell modus
        ultrasound.update_ultrasound_readings()

        x = y = omega = 0

        # Sett kommandoer manuelt eller via annen input
        # Dette er bare et eksempel:
        if modus == "manuell":
            # Sett testverdier her hvis ønskelig
            x = STEP
        else:
            fused_heading = heading_tracker.get_heading()
#            x, y, omega = autonom_logikk(fused_heading)

        current_command = (x, y, omega)
        if current_command != prev_command:
            send_movement_command(x, y, omega)
            prev_command = current_command

        # Oppdater heading
        fused_heading = heading_tracker.update()

        # Prosesser LIDAR-data
        ferske_lidar_points = [(angle, distance) for angle, distance in lidar.scan_data if distance > 0]

        # Du kan her skrive ut, logge eller prosessere data i stedet for å tegne
        # Eksempel:
        # print(f"Heading: {fused_heading}")
        # print(f"Ultralyd: {ultrasound.sensor_distances}")

        time.sleep(0.033)  # ~30 Hz

    send_movement_command(0, 0, 0.0)
    lidar.stop_lidar()

if __name__ == "__main__":
    main()
