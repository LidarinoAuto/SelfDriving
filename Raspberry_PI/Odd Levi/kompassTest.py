import smbus
import time
import math

# I2C-adresse for QMC5883L
QMC5883L_ADDRESS = 0x0D
bus = smbus.SMBus(1)

# Registeradresser
QMC5883L_CTRL1 = 0x09
QMC5883L_SET_RESET = 0x0B
QMC5883L_DATA = 0x00

def setup_qmc5883l():
    """Initialiserer QMC5883L med riktige innstillinger."""
    bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_SET_RESET, 0x01)  # Soft reset
    bus.write_byte_data(QMC5883L_ADDRESS, QMC5883L_CTRL1, 0b00011101)  # 10Hz, 128x oversampling, continuous
    time.sleep(0.5)

def twos_complement(val, bits):
    """Konverterer fra to-komplementsverdi."""
    if val & (1 << (bits - 1)):
        val -= (1 << bits)
    return val

def read_compass():
    """Leser r�data fra kompass og returnerer heading i grader."""
    data = bus.read_i2c_block_data(QMC5883L_ADDRESS, QMC5883L_DATA, 6)

    x = twos_complement((data[1] << 8) | data[0], 16)
    y = twos_complement((data[3] << 8) | data[2], 16)
    z = twos_complement((data[5] << 8) | data[4], 16)

    heading_rad = math.atan2(y, x)
    heading_deg = math.degrees(heading_rad)
    if heading_deg < 0:
        heading_deg += 360

    return heading_deg, x, y, z

if __name__ == "__main__":
    setup_qmc5883l()
    try:
        while True:
            heading, x, y, z = read_compass()
            print(f"Heading: {heading:.2f}�, X: {x}, Y: {y}, Z: {z}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Avslutter...")
