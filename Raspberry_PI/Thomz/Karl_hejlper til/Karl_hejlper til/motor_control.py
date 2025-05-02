# motor_control.py
"""
Motor controller som sender bevegelseskommandoer til ESP32 over seriell.
"""
import serial
import config
from logging_utils import setup_logger

logger = setup_logger(__name__)

def send_movement_command(x: float, y: float, omega: float):
    message = f"{x} {y} {omega:.2f}\n".encode()
    try:
        with serial.Serial(config.SERIAL_PORT,
                           config.SERIAL_BAUDRATE,
                           timeout=1) as ser:
            ser.write(message)
            logger.debug(f"Sendt til ESP32: {x} {y} {omega:.2f}")
    except Exception as e:
        logger.error(f"Feil ved sending til ESP32: {e}")

class MotorController:
    def set_speed(self, x: float, y: float, omega: float):
        send_movement_command(x, y, omega)

    def stop(self):
        send_movement_command(0.0, 0.0, 0.0)
        logger.info("Stopp-kommando sendt")

    def cleanup(self):
        pass