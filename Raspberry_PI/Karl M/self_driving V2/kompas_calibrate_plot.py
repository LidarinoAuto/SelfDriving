import smbus
import time
import matplotlib.pyplot as plt

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
    print("Roter roboten rundt flere runder. Avslutt med CTRL+C.")

    x_list, y_list = [], []

    plt.ion()
    fig, ax = plt.subplots()
    sc = ax.scatter([], [])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Kompass: r�data live-plot')
    ax.grid(True)
    ax.set_xlim(-1500, 1500)
    ax.set_ylim(-1500, 1500)
    plt.show()

    try:
        while True:
            x, y = read_raw_xy()
            x_list.append(x)
            y_list.append(y)
            sc.set_offsets(list(zip(x_list, y_list)))
            if len(x_list) % 50 == 0:  # Oppdater akseomr�de hver 50. punkt
                ax.set_xlim(min(x_list)-100, max(x_list)+100)
                ax.set_ylim(min(y_list)-100, max(y_list)+100)
            plt.draw()
            plt.pause(0.01)
    except KeyboardInterrupt:
        print("Plotting ferdig. Lukk vinduet for � avslutte.")
        plt.ioff()
        plt.show()

if __name__ == "__main__":
    main()
