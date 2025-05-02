# ultrasound_module.py
"""
Ultrasonic distance sensor using RPi.GPIO edge detection to minimize CPU usage.
"""
import RPi.GPIO as GPIO
import time
from logging_utils import setup_logger
import config

class UltrasoundModule:
    def __init__(self, trigger_pin=None, echo_pin=None):
        self.trigger = trigger_pin or config.ULTRASOUND_SENSOR_PINS['front_left_trigger']
        self.echo = echo_pin or config.ULTRASOUND_SENSOR_PINS['front_left_echo']
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)
        self.logger = setup_logger(self.__class__.__name__)

    def read_distance(self) -> float:
        """Perform a single ultrasonic distance measurement in cm."""
        GPIO.output(self.trigger, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trigger, GPIO.LOW)

        if GPIO.wait_for_edge(self.echo, GPIO.RISING, timeout=config.ULTRASOUND_TIMEOUT_MS):
            start = time.monotonic()
            if GPIO.wait_for_edge(self.echo, GPIO.FALLING, timeout=config.ULTRASOUND_TIMEOUT_MS):
                end = time.monotonic()
                duration = end - start
                return (duration * 34300) / 2
        self.logger.warning("Ultrasound read timeout")
        return float('nan')

    def cleanup(self):
        GPIO.cleanup()