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
            # F�r n� en LISTE over indekser til sensorer som trigget. Listen er tom hvis ingen trigget.
            triggered_ultrasound_sensors = check_ultrasound_all() # <-- Denne variabelen er korrekt her


            # --- Beslutningslogikk ---

            # Sjekk om det er en hindring fra LiDAR ELLER om ULTRALYD-listen IKKE er tom
            if lidar_distance <= 10 or triggered_ultrasound_sensors: # En liste er True hvis den ikke er tom
                print("Hindring detektert! Stopper og snur...")
                stop_robot()

                # --- Start Unn�vikelseslogikk ---
                # Bestem rotasjonsvinkelen basert p� hvilke sensorer som trigget

                turn_angle = 0 # Start med 0
                base_turn_angle = 45 # Standard svingvinkel for spesifikke US-triggere

                # Rekkef�lge i all_trig_pins (fra ultrasound_module): 0=FV, 1=FH, 2=BV, 3=BH

                # Spesifikk logikk basert p� hvilke front-sensorer (0 og 1) som trigget
                # Bruk KORREKT variabelnavn her: triggered_ultrasound_sensors
                is_fl_triggered = 0 in triggered_ultrasound_sensors # <-- Riktig navn
                is_fh_triggered = 1 in triggered_ultrasound_sensors # <-- Riktig navn
                is_back_triggered = any(idx in triggered_ultrasound_sensors for idx in [2, 3]) # <-- Riktig navn


                turn_angle = 0 # Start med 0
                base_turn_angle = 45 # Standard svingvinkel for spesifikke US-triggere


                # --- KORRIGERT LOGIKK FOR � SVINGE VEKK FRA HINDRINGEN (GITT FYSISK KOBLINGSBYTTE) ---
                # Hvis din fysisk venstre sensor (index 1) trigget, sving H�YRE
                if is_fh_triggered and not is_fl_triggered and not is_back_triggered:
                    turn_angle = base_turn_angle # Setter positiv vinkel (+45)
                    print("Hindring KUN Fysisk VENSTRE (index 1) -> Svinger h�yre") # Oppdatert print-tekst

                # Hvis din fysisk h�yre sensor (index 0) trigget, sving VENSTRE
                elif is_fl_triggered and not is_fh_triggered and not is_back_triggered:
                    turn_angle = -base_turn_angle # Setter NEGATIV vinkel (-45)
                    print("Hindring KUN Fysisk H�YRE (index 0) -> Svinger venstre") # Oppdatert print-tekst

                # --- SLUTT KORRIGERT LOGIKK ---


                elif is_fl_triggered or is_fh_triggered or is_back_triggered:
                    # B�DE Front Venstre (0) og Front H�yre (1) trigget, eller Bak (2/3) trigget, eller kombinasjon
                    print("ULTRALYD (kombinert/bak) -> Bruker fallback sving")
                    # turn_angle forblir 0, triggrer fallback under
                # ... (resten av logikken for fallback) ...


                # Hvis turn_angle fortsatt er 0 (betyr ingen spesifikk US-regel matchet, f.eks. LiDAR trigget alene,
                # eller begge front-US trigget, eller bak-US trigget, etc.):
                if turn_angle == 0:
                     print("Ingen spesifikk US-regel matchet ELLER KUN LiDAR -> Bruker vekslende sving")
                     # Bruk fallback vekslende sving
                     global last_turn_dir # M� declareres som global for � endre verdien
                     base_fallback_angle = 60 # Kan bruke annen vinkel for fallback, f.eks. st�rre sving
                     turn_angle = base_fallback_angle * last_turn_dir
                     last_turn_dir *= -1 # Bytt retning for neste fallback sving


                # Utf�r rotasjonen med den bestemte vinkelen
                # Pass p� at turn_angle ikke blir 0 hvis vi faktisk skal snu
                # Dette skal v�re h�ndtert av logikken, men en ekstra sjekk skader ikke
                if turn_angle != 0:
                    print(f"Roterer {turn_angle}� for � unnvike...")
                    rotate_by_gyro(turn_angle)
                    print("Rotasjon fullf�rt.")
                else:
                     print("Ingen rotasjon n�dvendig? Dette skal normalt ikke skje ved hindring.") # Skal i teorien ikke skje hvis if-betingelsen for hindring er sann


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
# Variabel for � veksle svingretning hvis fallback-logikken brukes
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