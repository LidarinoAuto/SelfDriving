# Filename: logging_utils.py
# Simple logging utility to write timestamps to console and file

import time
import os

def skriv_logg(melding):
    """
    Skriver en tidsstemplet melding til konsollen og en loggfil (robot_log.txt).
    """
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    full_melding = f"[{timestamp}] {melding}"

    # Skriv til konsollen
    print(full_melding)

    # Skriv til fil
    logg_filnavn = "robot_log.txt" # Gir den et mer spesifikt navn

    try:
        # Bruker 'a' for append-modus, legger til p� slutten av filen.
        # Lager filen om den ikke finnes.
        with open(logg_filnavn, "a") as f:
            f.write(full_melding + "\n")
    except IOError as e:
        # Hvis det skjer en feil med filskriving, print en feilmelding til konsollen.
        print(f"FEIL: Kunne ikke skrive til loggfilen {logg_filnavn}: {e}")
