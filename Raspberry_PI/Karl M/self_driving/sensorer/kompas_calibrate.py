# kompas_calibrate.py
import smbus
import time
import math

QMC5883L_ADDRESS = 0x0d
bus = smbus.SMBus(1)

QMC5883L_CTRL1 = 0x09
QMC5883L_SET_RESET = 0x0B
QMC5883L_DATA = 0x00

def init_compass():
    bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_SET_RESET, 0x01)
    time.sleep(0.1)
    bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_CTRL1, 0b00011101)
    time.sleep(0.1)

def read_raw_xy():
    data = bus.read_i2c_block_data(QMC5883L_ADDRESS, QMC5883L_DATA, 6)
    x_raw = (data[1] << 8) | data[0]
    y_raw = (data[3] << 8) | data[2]

    x = x_raw - 65536 if x_raw > 32767 else x_raw
    y = y_raw - 65536 if y_raw > 32767 else y_raw

    return x, y

def main():
    init_compass()
    print("Starter kompasskalibrering...")
    print("Roter roboten sakte rundt minst én hel runde.")
    print("Trykk CTRL+C når du er ferdig.")

    x_min = y_min = 32767
    x_max = y_max = -32768

    try:
        while True:
            x, y = read_raw_xy()
            x_min = min(x_min, x)
            x_max = max(x_max, x)
            y_min = min(y_min, y)
            y_max = max(y_max, y)

            print(f"x: {x}  y: {y}  |  x_min: {x_min}  x_max: {x_max}  y_min: {y_min}  y_max: {y_max}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nKalibrering ferdig!")
        offset_x = (x_min + x_max) / 2
        offset_y = (y_min + y_max) / 2
        print(f"Beregnet offset_x = {offset_x:.2f}")
        print(f"Beregnet offset_y = {offset_y:.2f}")

        # Lagre til fil
        with open("kompas_offset.txt", "w") as f:
            f.write(f"{offset_x}\n")
            f.write(f"{offset_y}\n")

        print("Offset-verdier lagret til kompas_offset.txt.")

if __name__ == "__main__":
    main()
