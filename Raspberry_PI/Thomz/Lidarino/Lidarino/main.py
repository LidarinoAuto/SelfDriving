# ------------------ IMPORTERTE MODULER ------------------
from lidar_module import start_lidar_thread, get_median_lidar_reading
from compass_module import init_compass, read_compass, get_compass_direction
from imu_module import init_imu, rotate_by_gyro
from ultrasound_module import setup_ultrasound, check_ultrasound_all, cleanup_ultrasound
from motor_control import move_forward, stop_robot
import time
import sys

# ------------------ GLOBALE VARIABLER ------------------
# Brukes for � veksle mellom h�yre og venstre fallback-sving
last_turn_dir = 1


# ------------------ HOVEDFUNKSJON ------------------
def main():
    print("Starter opp robotens systemer...")

    # Initialiser alle sensorer og tr�der
    try:
        setup_ultrasound()
        init_imu()
        init_compass()
        start_lidar_thread()
    except Exception as e:
        print(f"Feil under initialisering: {e}")
        sys.exit(1)

    print("Initialisering fullf�rt.")

    # Roter mot nord (0 grader)
    current_heading = read_compass()
    print(f"N�v�rende heading: {current_heading:.1f}�")

    target_heading = 0.0
    angle_to_rotate = (target_heading - current_heading + 360) % 360
    if angle_to_rotate > 180:
        angle_to_rotate -= 360

    print(f"Roterer {angle_to_rotate:.1f}� mot Nord...")
    rotate_by_gyro(angle_to_rotate)
    time.sleep(0.5)
    print(f"Ny heading etter rotasjon: {read_compass():.1f}�")

    # Start fremoverkj�ring
    print("Starter bevegelse fremover.")
    move_forward()

    # ------------------ HOVEDL�KKE ------------------
    try:
        while True:
            heading = read_compass()
            direction = get_compass_direction(heading)

            lidar_distance = get_median_lidar_reading()
            triggered_ultrasound_sensors = check_ultrasound_all()

            # Dersom en hindring er for n�r
            if lidar_distance <= 10 or triggered_ultrasound_sensors:
                print("Hindring oppdaget! Stopper og planlegger unnvikelse...")
                stop_robot()

                # Unnvikelseslogikk
                turn_angle = 0
                base_turn_angle = 45

                is_fl_triggered = 0 in triggered_ultrasound_sensors
                is_fh_triggered = 1 in triggered_ultrasound_sensors
                is_back_triggered = any(idx in triggered_ultrasound_sensors for idx in [2, 3])

                if is_fh_triggered and not is_fl_triggered and not is_back_triggered:
                    turn_angle = base_turn_angle
                    print("Venstre front-sensor trigget: svinger h�yre")
                elif is_fl_triggered and not is_fh_triggered and not is_back_triggered:
                    turn_angle = -base_turn_angle
                    print("H�yre front-sensor trigget: svinger venstre")
                elif is_fl_triggered or is_fh_triggered or is_back_triggered:
                    print("Flere sensorer trigget eller bak: fallback-rotasjon brukes")

                if turn_angle == 0:
                    global last_turn_dir
                    base_fallback_angle = 25
                    turn_angle = base_fallback_angle * last_turn_dir
                    last_turn_dir *= -1
                    print(f"Fallback-sving med {turn_angle}�")

                rotate_by_gyro(turn_angle)
                print("Unnvikelsesman�ver fullf�rt. Fortsetter fremover.")
                time.sleep(0.5)
                move_forward()

            # Kort pause mellom sykluser
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Brukeren avbr�t programmet (Ctrl+C).")


# ------------------ PROGRAMSTART ------------------
if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Uventet feil i hovedprogrammet: {e}")
        # traceback.print_exc()  # Aktiver ved behov for feils�king
    finally:
        print("Rydder opp f�r avslutning...")
        stop_robot()
        cleanup_ultrasound()
        # Legg til opprydding for flere moduler ved behov
        print("Programmet er avsluttet.")
