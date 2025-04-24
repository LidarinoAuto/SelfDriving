# Filnavn: main.py

# Importer n�dvendige moduler og funksjoner
from lidar_module import start_lidar_thread, get_median_lidar_reading
from compass_module import init_compass, read_compass, get_compass_direction # get_compass_direction brukes n�
from imu_module import init_imu, rotate_by_gyro # rotate_by_gyro trengs her
# Importerer setup, check og cleanup fra ultralydmodulen
from ultrasound_module import setup_ultrasound, check_ultrasound_all, cleanup_ultrasound
from motor_control import move_forward, stop_robot # move_forward og stop_robot trengs her
import time
import sys # Trengs for � h�ndtere opprydding ved avslutning
# import traceback # Nyttig for mer detaljert feilrapportering

# Variabel for � veksle svingretning hvis fallback-logikken brukes
# M� DEFINERES UTENFOR FUNKSJONER og M� IKKE V�RE KOMMENTERT UT!
last_turn_dir = 1 # Initial retning for veksling (1 for h�yre, -1 for venstre)


def main():
    print("Starter opp robotens systemer...")
    # Initialiser de forskjellige moduler
    try:
        setup_ultrasound()
        init_imu()
        init_compass()
        start_lidar_thread() # Start LiDAR-tr�den i bakgrunnen
    except Exception as e:
        print(f"Feil under initialisering: {e}")
        # Vurder om programmet skal avsluttes her eller pr�ve � fortsette
        # Viktig: Hvis initialisering feiler, kan sensorer v�re ustabile!
        sys.exit(1) # Avslutt programmet hvis initialisering feiler

    print("Initialisering fullf�rt.")

    # Eksempel: Roter roboten mot nord (0 grader) basert p� kompass
    # Finn n�v�rende kompass-heading
    current_heading = read_compass()
    print(f"N�v�rende heading: {current_heading:.1f}�")

    # Beregn vinkel som trengs for � rotere til Nord (0 grader)
    target_heading = 0.0 # Nord
    # Beregn korteste vei � rotere. (M�l - N�v�rende). Modulo 360 for � holde innenfor en sirkel.
    angle_to_rotate = (target_heading - current_heading + 360) % 360
    if angle_to_rotate > 180:
        angle_to_rotate -= 360 # Hvis veien over 180, ta den andre veien (negativ vinkel)

    print(f"Roterer {angle_to_rotate:.1f}� mot Nord (0�)...")
    # Kall rotate_by_gyro med vinkelen som skal roteres
    rotate_by_gyro(angle_to_rotate)
    # Gi kompasset et �yeblikk til � stabilisere seg etter rotasjon hvis n�dvendig
    time.sleep(0.5)
    # Sjekk ny heading etter rotasjon
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
            # Kompasset kan brukes for mer avansert navigasjon eller holde retning
            heading = read_compass()
            direction = get_compass_direction(heading)
            # print(f"Kompass: {heading:.1f}� ({direction})") # Valgfri utskrift

            # Sjekk LiDAR for hindringer foran (under 10 cm)
            lidar_distance = get_median_lidar_reading()
            # print(f"LiDAR: {lidar_distance:.1f} cm") # Valgfri utskrift

            # Sjekk ULTRALYD for hindringer (under 15 cm i ultrasound_module).
            # F�r n� indeksen til sensoren som trigget, eller -1.
            ultrasound_sensor_index = check_ultrasound_all()


            # --- Beslutningslogikk ---

            # Sjekk om det er en hindring fra LiDAR ELLER en hindring fra Ultralyd
            if lidar_distance <= 10 or ultrasound_sensor_index != -1:
                print("Hindring detektert! Stopper og snur...")
                stop_robot()

                # --- Start Unn�vikelseslogikk ---
                # Bestem rotasjonsvinkelen basert p� sensoren som trigget

                turn_angle = 0 # Start med 0, s� vet vi om en spesifikk US-regel ble brukt
                base_turn_angle = 45 # Standard svingvinkel for spesifikke US-triggere (f.eks. 45 eller 90 grader)


                # Hvis en ULTRALYDsensor trigget:
                if ultrasound_sensor_index != -1:
                    print(f"Trigget av ultralydsensor index: {ultrasound_sensor_index}")
                    # Rekkef�lge i all_trig_pins (fra ultrasound_module): 0=FV, 1=FH, 2=BV, 3=BH

                    if ultrasound_sensor_index == 0: # Front Venstre trigget
                        # Sving til h�yre (positiv vinkel)
                        turn_angle = base_turn_angle
                        print("Hindring FV -> Svinger h�yre")
                    elif ultrasound_sensor_index == 1: # Front H�yre trigget
                         # Sving til venstre (negativ vinkel)
                        turn_angle = -base_turn_angle
                        print("Hindring FH -> Svinger venstre")
                    elif ultrasound_sensor_index == 2 or ultrasound_sensor_index == 3: # Bak sensor trigget
                         # Dette kan indikere et problem (kj�rer baklengs inn i noe?).
                         # Kanskje rygge litt f�rst, eller snu 180 grader?
                         # For n�, la oss bare bruke fallback-logikken (vekslende sving).
                         print("Hindring bak -> Bruker fallback sving")
                         pass # turn_angle forblir 0, triggrer fallback under

                # Hvis INGEN spesifikk ultralydregel ble brukt (enten LiDAR trigget,
                # eller en baksensor trigget, eller ingen US trigget men LiDAR gjorde det):
                # turn_angle er fortsatt 0 hvis ingen front-US regel matchet
                if turn_angle == 0:
                     print("LiDAR trigget eller Bak-US/Ingen spesifikk US -> Bruker vekslende sving")
                     # Bruk fallback vekslende sving
                     global last_turn_dir # M� declareres som global for � endre verdien
                     base_fallback_angle = 60 # Kan bruke annen vinkel for fallback, f.eks. st�rre sving
                     turn_angle = base_fallback_angle * last_turn_dir
                     last_turn_dir *= -1 # Bytt retning for neste fallback sving


                # Utf�r rotasjonen med den bestemte vinkelen
                print(f"Roterer {turn_angle}� for � unnvike...")
                rotate_by_gyro(turn_angle)
                print("Rotasjon fullf�rt.")

                # Gi sensorene et �yeblikk til � stabilisere seg etter sving
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
# M� DEFINERES UTENFOR FUNKSJONER og M� IKKE V�RE KOMMENTERT UT!
last_turn_dir = 1 # Initial retning for veksling (1 for h�yre, -1 for venstre)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Det oppstod en uventet feil i hovedprogrammet: {e}")
        # Inkluder feilmeldingen i oppryddingskonteksten
        # import traceback # Uncomment this import if you uncomment the line below
        # traceback.print_exc() # Skriver ut detaljert stacktrace - nyttig for feils�king
    finally:
        # --- Opprydding ved programslutt ---
        print("Starter opprydding...")
        stop_robot() # Forsikre deg om at roboten stopper
        cleanup_ultrasound() # Rydd opp i GPIO for ultralyd
        # TODO: Legg til cleanup for andre moduler her om n�dvendig (f.eks. lidar.stop(), imu.cleanup())
        print("Programmet er avsluttet.")