import RPi.GPIO as GPIO 

import serial 

import time 

import threading 

import logging 

from rplidar import RPLidar 

import smbus 

import math 

 

# Konfigurer logging 

logging.basicConfig(level=logging.INFO) 

 

# ----------------- KONSTANTER ----------------- 

PORT_NAME = "/dev/ttyUSB1"         # Port for RPLidar 

ESP_PORT = "/dev/ttyUSB0"           # Port for ESP32 

LIDAR_BAUDRATE = 115200 

SERIAL_BAUDRATE = 115200 

 

# Hindringsdeteksjon 

STOP_DISTANCE_LIDAR = 250          # mm 

STOP_DISTANCE_ULTRASOUND = 15      # cm 

 

# Bevegelsesparametere (verdier i mm/s og rad/s) 

SPEED = 100                      # mm/s (standard fremoverhastighet) 

ROTATION_SPEED = 2               # Rotasjonshastighet for eventuelle justeringer 

FORWARD_DISTANCE_AFTER_AVOID = 50   # mm (avstand etter unnamaning) 

TIME_TO_MOVE_FORWARD = FORWARD_DISTANCE_AFTER_AVOID / SPEED  # sekunder 

 

# For omni-baserte sidestreifinger 

AVOID_SPEED = 100              # mm/s (bruk samme som SPEED) 

AVOID_MOVE_DISTANCE = 50       # mm (avstand for sidelengs bevegelse) 

TIME_TO_STRAFE = AVOID_MOVE_DISTANCE / AVOID_SPEED  # sekunder 

 

# Ultralydsensor-pinner (rekkef�lge: front_left, front_right, back_left, back_right) 

ULTRASOUND_TRIG_PINS = [9, 7, 23, 10] 

ULTRASOUND_ECHO_PINS = [8, 6, 24, 11] 

ULTRASOUND_SENSOR_NAMES = ["front_left", "front_right", "back_left", "back_right"] 

 

# Mapping av ultrasonicsensorer til monteringsvinkler (i radianer) 

# For eksempel: front_left: 45�, front_right: -45�, back_left: 135�, back_right: -135� 

ULTRASOUND_SENSOR_ANGLES = { 

    0: math.radians(45), 

    1: math.radians(-45), 

    2: math.radians(135), 

    3: math.radians(-135) 

} 

 

# IMU-konfigurasjon (MPU6050) 

IMU_ADDRESS = 0x68 

GYRO_SENSITIVITY = 131.0         # LSB/(�/s) 

 

# ----------------- ULTRALYDSENSOR ----------------- 

class UltrasoundSensor: 

    def __init__(self, trig_pins, echo_pins, sensor_names): 

        self.trig_pins = trig_pins 

        self.echo_pins = echo_pins 

        self.sensor_names = sensor_names 

        self.setup() 

 

    def setup(self): 

        GPIO.setmode(GPIO.BCM) 

        for trig in self.trig_pins: 

            GPIO.setup(trig, GPIO.OUT) 

            GPIO.output(trig, False) 

        for echo in self.echo_pins: 

            GPIO.setup(echo, GPIO.IN) 

        time.sleep(2) 

 

    def read_distance(self, trig, echo): 

        GPIO.output(trig, False) 

        time.sleep(2e-6) 

        GPIO.output(trig, True) 

        time.sleep(10e-6) 

        GPIO.output(trig, False) 

     

        timeout_start = time.time() 

        while GPIO.input(echo) == 0: 

            pulse_start = time.time() 

            if pulse_start - timeout_start > 0.03:  # Timeout etter 30 ms 

                return -1 

 

        timeout_start = time.time() 

        while GPIO.input(echo) == 1: 

            pulse_end = time.time() 

            if pulse_end - timeout_start > 0.03: 

                return -1 

 

        duration = (pulse_end - pulse_start) * 1e6  # Mikrosekunder 

        if duration >= 38000: 

            return -1 

        else: 

            return duration / 58.0  # Avstand i cm 

 

    def get_distances(self, sensor_indices=None): 

        distances = [] 

        indices = sensor_indices if sensor_indices is not None else range(len(self.trig_pins)) 

        for i in indices: 

            d = self.read_distance(self.trig_pins[i], self.echo_pins[i]) 

            distances.append((i, d)) 

            if d > 0: 

                logging.info(f"{self.sensor_names[i]}: {d:.2f} cm") 

            else: 

                logging.info(f"{self.sensor_names[i]}: ingen m�ling") 

        return distances 

 

    def get_best_direction(self, sensor_indices=None): 

        distances = self.get_distances(sensor_indices) 

        best_index = None 

        best_distance = -1 

        for i, d in distances: 

            if d > best_distance: 

                best_distance = d 

                best_index = i 

        return best_index, best_distance 

 

    def check_obstacle(self, stop_distance, sensor_indices=None): 

        distances = self.get_distances(sensor_indices) 

        for i, d in distances: 

            if d > 0 and d < stop_distance: 

                logging.info(f"Hindring oppdaget av sensor {i} ({self.sensor_names[i]}) p� {d:.2f} cm") 

                return True 

        return False 

 

# ----------------- LIDAR-H�NDTERING ----------------- 

class LiDARHandler: 

    def __init__(self, port, baudrate=LIDAR_BAUDRATE): 

        self.port = port 

        self.baudrate = baudrate 

        self.lidar = RPLidar(self.port, baudrate=self.baudrate) 

        self.current_distance = 9999 

        self.running = True 

        self.thread = threading.Thread(target=self.lidar_loop) 

        self.thread.daemon = True 

 

    def start(self): 

        self.thread.start() 

 

    def lidar_loop(self): 

        for scan in self.lidar.iter_scans(): 

            if not self.running: 

                break 

            # Filtrer m�linger rett foran roboten (ca. �30�) 

            front_distances = [ 

                measurement[2] 

                for measurement in scan 

                if abs(measurement[1]) <= 30 or abs(measurement[1]-360) <= 30 

            ] 

            if front_distances: 

                self.current_distance = min(front_distances) 

            time.sleep(0.01) 

 

    def stop(self): 

        self.running = False 

        self.lidar.stop() 

        self.lidar.disconnect() 

 

# ----------------- IMU-H�NDTERING ----------------- 

class IMUHandler: 

    def __init__(self, address=IMU_ADDRESS, bus_number=1): 

        self.address = address 

        self.bus = smbus.SMBus(bus_number) 

        self.init_imu() 

 

    def init_imu(self): 

        self.bus.write_byte_data(self.address, 0x6B, 0) 

        time.sleep(0.1) 

 

    def read_word(self, adr): 

        high = self.bus.read_byte_data(self.address, adr) 

        low = self.bus.read_byte_data(self.address, adr + 1) 

        return (high << 8) + low 

 

    def read_word_2c(self, adr): 

        val = self.read_word(adr) 

        return -((65535 - val) + 1) if val >= 0x8000 else val 

 

    def get_gyro_z(self): 

        gyro_z = self.read_word_2c(0x47) 

        return gyro_z / GYRO_SENSITIVITY 

 

# ----------------- MOTORSTYRING ----------------- 

class MotorController: 

    def __init__(self, esp_port, baudrate=SERIAL_BAUDRATE): 

        self.esp = serial.Serial(esp_port, baudrate, timeout=1) 

        time.sleep(2)  # Gi ESP32 tid til oppstart 

 

    def send_command(self, v_x, v_y, r): 

        command = f"{v_x} {v_y} {r}\n" 

        logging.info(f"Sender kommando til ESP32: {command.strip()}") 

        self.esp.write(command.encode()) 

 

    def close(self): 

        self.esp.close() 

 

# ----------------- ROBOTKONTROLLER ----------------- 

class RobotController: 

    def __init__(self): 

        self.ultrasound = UltrasoundSensor( 

            ULTRASOUND_TRIG_PINS, ULTRASOUND_ECHO_PINS, ULTRASOUND_SENSOR_NAMES 

        ) 

        self.lidar_handler = LiDARHandler(PORT_NAME) 

        self.imu = IMUHandler() 

        self.motor = MotorController(ESP_PORT) 

 

    def rotate_by_angle(self, angle): 

        rotation_time = (abs(angle) / 90.0) * 1.5  # Basert p� 1.5 s for 90� 

        if angle < 0: 

            self.motor.send_command(0, 0, ROTATION_SPEED) 

        else: 

            self.motor.send_command(0, 0, -ROTATION_SPEED) 

        start_time = time.time() 

        last_time = start_time 

        integrated_angle = 0.0 

        while time.time() - start_time < rotation_time: 

            current_time = time.time() 

            dt = current_time - last_time 

            last_time = current_time 

            angular_rate = self.imu.get_gyro_z() 

            integrated_angle += angular_rate * dt 

            time.sleep(0.01) 

        self.motor.send_command(0, 0, 0) 

        logging.info(f"Integrert rotasjon (m�lt av gyro): {integrated_angle:.2f} grader") 

 

    def choose_direction_and_strafe(self): 

        best_index, best_distance = self.ultrasound.get_best_direction() 

        if best_index is None: 

            logging.info("Ingen sensor ga p�litelig verdi. Bruker standard fremover.") 

            return SPEED, 0 

        sensor_angle = ULTRASOUND_SENSOR_ANGLES.get(best_index, 0) 

        v_x = AVOID_SPEED * math.cos(sensor_angle) 

        v_y = AVOID_SPEED * math.sin(sensor_angle) 

        logging.info(f"Valgt retning: sensor {best_index} (vinkel: {math.degrees(sensor_angle):.1f}�), beveg: {v_x:.1f} mm/s, {v_y:.1f} mm/s") 

        return v_x, v_y 

 

    def move_forward(self): 

        logging.info("Kj�rer fremover") 

        self.motor.send_command(SPEED, 0, 0) 

 

    def move_strafe(self, v_x, v_y, duration): 

        logging.info(f"Utf�rer sidelengs bevegelse: v_x={v_x:.1f} mm/s, v_y={v_y:.1f} mm/s i {duration:.2f} s") 

        self.motor.send_command(v_x, v_y, 0) 

        time.sleep(duration) 

        self.motor.send_command(0, 0, 0) 

 

    def stop_robot(self): 

        logging.info("Stopper roboten") 

        self.motor.send_command(0, 0, 0) 

 

    def move_forward_distance(self): 

        logging.info("Kj�rer et kort stykke fremover etter unnamaning") 

        self.motor.send_command(SPEED, 0, 0) 

        time.sleep(TIME_TO_MOVE_FORWARD) 

        self.motor.send_command(0, 0, 0) 

 

    def avoid_obstacle(self): 

        self.stop_robot() 

        time.sleep(1) 

        v_x, v_y = self.choose_direction_and_strafe() 

        self.move_strafe(v_x, v_y, TIME_     -import RPi.GPIO as GPIO 

import serial 

import time 

import threading 

import logging 

from rplidar import RPLidar 

import smbus 

import math 

 

# Konfigurer logging 

logging.basicConfig(level=logging.INFO) 

 

# ----------------- KONSTANTER ----------------- 

PORT_NAME = "/dev/ttyUSB1"         # Port for RPLidar 

ESP_PORT = "/dev/ttyUSB0"           # Port for ESP32 

LIDAR_BAUDRATE = 115200 

SERIAL_BAUDRATE = 115200 

 

# Hindringsdeteksjon 

STOP_DISTANCE_LIDAR = 250          # mm 

STOP_DISTANCE_ULTRASOUND = 15      # cm 

 

# Bevegelsesparametere (verdier i mm/s og rad/s) 

SPEED = 100                      # mm/s (standard fremoverhastighet) 

ROTATION_SPEED = 2               # Rotasjonshastighet for eventuelle justeringer 

FORWARD_DISTANCE_AFTER_AVOID = 50   # mm (avstand etter unnamaning) 

TIME_TO_MOVE_FORWARD = FORWARD_DISTANCE_AFTER_AVOID / SPEED  # sekunder 

 

# For omni-baserte sidestreifinger 

AVOID_SPEED = 100              # mm/s (bruk samme som SPEED) 

AVOID_MOVE_DISTANCE = 50       # mm (avstand for sidelengs bevegelse) 

TIME_TO_STRAFE = AVOID_MOVE_DISTANCE / AVOID_SPEED  # sekunder 

 

# Ultralydsensor-pinner (rekkef�lge: front_left, front_right, back_left, back_right) 

ULT