import time

def skriv_logg(melding, filnavn="logg.txt"):
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    full_melding = f"[{timestamp}] {melding}"
    print(full_melding)  # Printer i terminalen ogs�

    with open(filnavn, "a") as f:
        f.write(full_melding + "\n")
