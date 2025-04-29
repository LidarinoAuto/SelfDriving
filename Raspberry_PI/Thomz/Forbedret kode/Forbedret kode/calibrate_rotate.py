# Filename: calibrate_rotate.py

import time
import math
# Importer Headin_Tracker klassen (kan vre n dvendig for initialisering/opprydding)
from Heading_Tracker import HeadingTracker 
# Importer funksjonene fra motor_control modulen
from motor_control import send_command, stop_robot 
# Importer logging funksjonen
from logging_utils import skriv_logg

# --- TEMPOR RE IMPORTER FOR SENSOR INITIALISERING OG LESING ---
# DISSE M  MATCHE HVORDAN DU FAKTISK INITIALISERER OG LESER SENSORER I DITT PROSJEKT!
# Juster stier og funksjonsnavn slik at de stemmer for dine moduler (f.eks. imu_module.py, compass_module.py)
try:
  # Eksempel: Antar at init_imu og read_imu_data finnes i imu_module.py
  from imu_module import init_imu, read_imu_data 
  # Eksempel: Antar at init_compass og read_compass_data finnes i compass_module.py
  from compass_module import init_compass, read_compass_data 
  SENSOR_IMPORTS_OK = True
except ImportError as e:
  skriv_logg(f"[CALIB] Advarsel: Kunne ikke importere sensor funksjoner: {e}")
  skriv_logg("[CALIB] Sjekk importene i calibrate_rotate.py og juster dem til ditt prosjekt.")
  SENSOR_IMPORTS_OK = False
  
# --- Konfigurasjon for kalibrering ---
TEST_DURATION = 10  # Sekunder roboten skal rotere
# Kommando verdi som tilsvarer din base_speed_heading (f.eks. 50)
CALIBRATION_COMMAND_VALUE = 50.0 # Bruk 50.0 for flyttall til ESP32


# --- Initialiser systemer ---
# Initialiserer n dvendige deler: sensorer for HeadingTracker og s rger for a motorkontroll funksjonene er klare
heading_tracker = None # Initialiser HeadingTracker objektet til None i tilfelle feil

try:
  skriv_logg("[CALIB] Initialiserer systemer...")

  if SENSOR_IMPORTS_OK:
    # --- Initialiser sensorer slik du gj r i main.py ---
    # Kall dine faktiske initialiseringsfunksjoner her:
    skriv_logg("[CALIB] Initialiserer IMU...")
    if init_imu(): 
      skriv_logg("[CALIB] IMU initialisert.")
    else:
      skriv_logg("[CALIB] FEIL: IMU initialisering mislyktes.")
      raise RuntimeError("IMU initialization failed") # Avbryt om kritisk sensor feiler
      
    skriv_logg("[CALIB] Initialiserer Compass...")
    if init_compass(): 
      skriv_logg("[CALIB] Compass initialisert.")
    else:
      skriv_logg("[CALIB] FEIL: Compass initialisering mislyktes.")
      raise RuntimeError("Compass initialization failed") # Avbryt om kritisk sensor feiler

    time.sleep(1) # Gi litt tid til sensorer

    # --- Initialiser HeadingTracker ---
    # Initialiser HeadingTracker ved a sende inn de faktiske lese-funksjonene
    heading_tracker = HeadingTracker(read_imu_data, read_compass_data) 
    skriv_logg("[CALIB] HeadingTracker initialisert.")

  else:
    skriv_logg("[CALIB] Kan ikke initialisere sensorer pga importfeil. Avbryter.")
    exit() # Avslutt programmet om sensorer ikke kan importeres/initialiseres


  time.sleep(2) # Gi tid til initialisering og for at HeadingTracker skal sette f rste heading
  skriv_logg("[CALIB] Initialisering fullfort.")

  # --- Kj r kalibrering ---
  skriv_logg(f"[CALIB] Starter rotasjonskalibrering med kommando {CALIBRATION_COMMAND_VALUE:.2f} i {TEST_DURATION} sekunder ved   integrere gyro data...")

  # *** Viktig: Ignore de vanlige PID Debug linjene i loggen for n ***
  # *** Du skal SE P  SLUTTRESULTATET som skrives ut av dette skriptet ***

  # Gi en stoppkommando for sikkerhets skyld ved a kalle funksjonen direkte
  send_command(f"0.0 0.0 0.0\n") 
  time.sleep(0.5)

  # Vent til brukeren er klar
  input("[CALIB] Trykk ENTER n r du er klar til   starte rotasjonen og overv ke loggen for resultat.")
  
  # --- Rotasjonssl yfe med Gyro-integrasjon ---
  skriv_logg(f"[CALIB] Roterer i {TEST_DURATION} sekunder og integrerer gyro data...") 
  start_time = time.time()
  last_loop_time = time.time() # For a beregne dt
  total_gyro_rotation = 0.0 # Akkumulerer rotasjon i grader

  while (time.time() - start_time) < TEST_DURATION:
    current_loop_time = time.time()
    dt = current_loop_time - last_loop_time # Tiden som har g tt siden sist loop kj rte
    last_loop_time = current_loop_time # Oppdater sist tid

    # Send rotasjonskommandoen som flyttall (vx=0, vy=0, omega=kommando)
    command_string = f"0.0 0.0 {CALIBRATION_COMMAND_VALUE:.2f}\n" 
    send_command(command_string)

    # Les r gyro data i grader/sekund
    # Antar read_imu_data() returnerer Z-akse angular velocity i deg/s
    gyro_rate = read_imu_data() 

    # Integrer gyro rate over dt for a f total rotasjon i grader i dette tidsintervallet
    # Srg for at dt ikke er null for a unng feil
    if dt > 0:
      gyro_delta_rotation = gyro_rate * dt 
      total_gyro_rotation += gyro_delta_rotation # Akkumuler totalen

    # En kort pause for a unng overbelastning og sikre meningsfull dt
    time.sleep(0.01) # Kan justeres for a p virke frekvensen p dt


  # --- Stop roboten etter testperioden ---
  skriv_logg("[CALIB] Stopper rotasjonen...")
  send_command(f"0.0 0.0 0.0\n")
  time.sleep(1) # Gi tid til at roboten stopper fysisk

  # --- Beregn og rapporter resultat ---
  # N vet vi den totale rotasjonen basert p gyro integrering over testperioden
  # Beregn gjennomsnittlig rotasjonshastighet
  # Srg for at testvarigheten ikke er null for a unng feil
  measured_deg_per_s = 0.0
  if TEST_DURATION > 0:
    measured_deg_per_s = total_gyro_rotation / TEST_DURATION

  skriv_logg(f"[CALIB] --- KALIBRERING RESULTAT ---")
  skriv_logg(f"[CALIB] Testvarighet: {TEST_DURATION} sekunder")
  skriv_logg(f"[CALIB] Kommando sendt til ESP32 (omega): {CALIBRATION_COMMAND_VALUE:.2f}")
  skriv_logg(f"[CALIB] Total rotasjon basert p  Gyro-integrasjon: {total_gyro_rotation:.2f} grader")
  skriv_logg(f"[CALIB] Gjennomsnittlig rotasjonshastighet: {measured_deg_per_s:.2f} grader/sekund")
  skriv_logg(f"[CALIB] ---")
  skriv_logg(f"[CALIB] Bruk verdien {measured_deg_per_s:.2f} som din MAX_ROBOT_ROTATION_DEGPS_AT_BASE_SPEED i imu_module.py")


except Exception as e:
  skriv_logg(f"[CALIB] En feil oppstod: {e}")

finally:
  # --- Rydder opp (viktig!) ---
  skriv_logg("[CALIB] Rydder opp...")
  # Srg for at motorer stopper uansett feil
  try:
    send_command(f"0.0 0.0 0.0\n")
    # Legg til logikk for a lukke serieforbindelsen om nodvendig (avhenger av motor_control.py)
    # Hvis din send_command hndterer en global 'esp' variabel:
    # import serial # m importeres hvis ikke allerede gjort
    # if 'esp' in globals() and isinstance(esp, serial.Serial) and esp.isOpen():
    #    esp.close()
  except Exception as e:
    skriv_logg(f"[CALIB] Feil ved sending av stoppkommando under opprydding: {e}")
  
  # Stopp HeadingTracker tr den om den kjrer i egen tr d
  if heading_tracker: # Sjekk om objektet ble opprettet
    # Hvis HeadingTracker har stop_thread og join_thread metoder:
    # try:
    #    heading_tracker.stop_thread()
    #    heading_tracker.join_thread()
    # except AttributeError:
    #    pass # Metoder finnes ikke
    pass # Placeholder hvis HeadingTracker ikke kjrer tr d
  
  skriv_logg("[CALIB] Opprydding fullfort. Avslutter kalibreringsskript.")