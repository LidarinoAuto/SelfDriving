import serial
import time

def main():
    # Endre denne porten slik at den peker til Arduinoens Serial2-tilkobling
    ARDUINO_SERIAL2_PORT = "/dev/ttyUSB0"  
    BAUD_RATE = 115200

    try:
        ser = serial.Serial(ARDUINO_SERIAL2_PORT, BAUD_RATE, timeout=1)
        print(f"Lytter p� {ARDUINO_SERIAL2_PORT} med {BAUD_RATE} baud")
    except Exception as e:
        print("Kunne ikke �pne seriekoblingen:", e)
        return

    try:
        while True:
            # Les linje hvis data er tilgjengelig
            if ser.in_waiting:
                data = ser.readline()
                try:
                    decoded_data = data.decode('utf-8').strip()
                except Exception as e:
                    print("Feil ved dekoding:", e)
                    continue
                if decoded_data:
                    print("Mottatt data:", decoded_data)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nAvslutter programmet...")
    finally:
        ser.close()
        print("Seriekoblingen er lukket.")

if __name__ == "__main__":
    main()
