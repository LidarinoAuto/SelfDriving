# Filnavn: main.py

# Importer n�dvendige moduler og funksjoner
from lidar_module import start_lidar_thread, get_median_lidar_reading
from compass_module import init_compass, read_compass, get_compass_direction
from imu_module import init_imu, rotate_by_gyro # rotate_by_gyro trengs her
# Importerer setup, check og cleanup fra ultralydmodulen
from ultrasound_module import setup_ultrasound, check_ultrasound_all, cleanup_ultrasound
from motor_control import move_forward, stop_robot # move_forward og stop_robot trengs her
import time
import sys # Trengs for � h�ndtere opprydding ved avslutning

def main():
    print("Starter opp robotens systemer...")
    # Initialiser de forskjellige modulene
    try:
        setup_ultrasound()
        init_imu()
        init_compass()
        start_lidar_thread() # Start LiDAR-tr�den i bakgrunnen
    except Exception as e:
        print(f"Feil under initialisering: {e}")
        # Vurder om programmet skal avsluttes her eller pr�ve � fortsette
        sys.exit(1) # Avslutt programmet hvis initialisering feiler

    print("Initialisering fullf�rt.")

    # Eksempel: Roter roboten mot nord (0 grader) basert p� kompass
    # Finn n�v�rende kompass-heading
    current_heading = read_compass()
    print(f"N�v�rende heading: {current_heading:.1f}�")

    # Beregn vinkel som trengs for � rotere til Nord (0 grader)
    target_heading = 0.0 # Nord
    angle_to_rotate = (target_heading - current_heading + 360) % 360
    if angle_to_rotate > 180:
        angle_to_rotate -= 360 # Velg den korteste veien

    print(f"Roterer {angle_to_rotate:.1f}� mot Nord (0�)...")
    # Kall rotate_by_gyro med vinkelen som skal roteres
    rotate_by_gyro(angle_to_rotate)
    # Gi kompasset et �yeblikk til � stabilisere seg etter rotasjon hvis n�dvendig
    time.sleep(0.5)
    print(f"Rotasjon mot Nord fullf�rt. Ny heading: {read_compass():.1f}�")


    # Start bevegelse fremover
    print("Starter bevegelse fremover...")
    move_forward()

    # Hovedl�kke for navigasjon og hindringsdeteksjon
    print("Starter hovednavigasjonsl�kke.")
    try:
        while True:
            # --- Overv�kning av sensorer ---

            # Sjekk kompass (valgfritt i l�kken med mindre du navigerer kontinuerlig)
            heading = read_compass()
            direction = get_compass_direction(heading)
            print(f"Kompass: {heading:.1f}� ({direction})")

            # Sjekk LiDAR for hindringer foran (under 10 cm)
            lidar_distance = get_median_lidar_reading()
            print(f"LiDAR: {lidar_distance:.1f} cm") # Valgfri utskrift

            # Sjekk ULTRALYD for hindringer (under 15 cm i ultrasound_module)
            ultrasound_obstacle_detected = check_ultrasound_all()

            # --- Beslutningslogikk ---

            # Hvis LiDAR eller Ultralyd detekterer en n�r hindring
            if lidar_distance <= 10 or ultrasound_obstacle_detected:
                print("Hindring detektert! Stopper og snur...")
                stop_robot()

                # --- Start Unn�vikelseslogikk ---
                # Definer hvor mange grader roboten skal snu
                # turn_angle = 20 # Eksempel: sving 45 grader til h�yre
                # For � veksle retning hver gang, kan du bruke en variabel utenfor l�kken:
                global last_turn_dir
                turn_angle = 15 * last_turn_dir
                last_turn_dir *= -1 # Bytt retning for neste gang (m� initialiseres f�r l�kken, f.eks. last_turn_dir = 1)

                print(f"Roterer {turn_angle}� for � unnvike...")
                # Kall rotate_by_gyro med vinkelen som skal roteres
                rotate_by_gyro(turn_angle)
                print("Rotasjon fullf�rt.")

                # Gi sensorene (spesielt kompasset hvis det brukes i l�kken) et �yeblikk til � stabilisere seg
                time.sleep(0.5) # Gi sensorer og robot tid etter sving

                # Start � kj�re fremover igjen etter svingen
                print("Fortsetter bevegelse fremover.")
                move_forward()
                # --- Slutt Unn�vikelseslogikk ---

                # VIKTIG: Fjern "break" som stopper l�kken helt.
                # L�kken vil n� fortsette i den nye retningen.
                # break

            # --- Vent litt f�r neste sensoravlesningssyklus ---
            time.sleep(0.1) # Kort pause for � ikke overbelaste CPU og sensorer

    except KeyboardInterrupt:
        # Dette fanges i if __name__ blokken
        print("Hovedl�kke avbrutt av bruker (Ctrl+C).")
    # Her vil 'finally'-blokken under h�ndtere opprydding uansett hvordan l�kken avsluttes (break eller Ctrl+C)


# --- Programmet starter her ---
# Variabel for � veksle svingretning hvis �nskelig (m� defineres utenfor funksjoner)
last_turn_dir = 1 # Initial retning for veksling (1 for h�yre, -1 for venstre)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Det oppstod en uventet feil i hovedprogrammet: {e}")
        # Inkluder feilmeldingen i oppryddingskonteksten
        # import traceback
        # traceback.print_exc()
    finally:
        # --- Opprydding ved programslutt ---
        print("Starter opprydding...")
        stop_robot() # Forsikre deg om at roboten stopper
        cleanup_ultrasound() # Rydd opp i GPIO for ultralyd
        # TODO: Legg til cleanup for andre moduler her om n�dvendig (f.eks. lidar.stop(), imu.cleanup())
        print("Programmet er avsluttet.")