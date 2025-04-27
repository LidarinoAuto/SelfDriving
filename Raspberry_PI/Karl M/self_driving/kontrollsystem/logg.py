import time

def skriv_logg(melding):
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    full_melding = f"[{timestamp}] {melding}"
    print(full_melding)  # Printer i terminalen ogs�

    with open("logg.txt", "a") as f:
        f.write(full_melding + "\n")
