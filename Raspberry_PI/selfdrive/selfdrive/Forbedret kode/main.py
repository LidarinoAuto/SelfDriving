import pygame
import serial
import time
import math
import threading
import os

# ======== LOGGING ========
def skriv_logg(melding, filnavn="logg.txt"):
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    full_melding = f"[{timestamp}] {melding}"
    print(full_melding)
    with open(filnavn, "a") as f:
        f.write(full_melding + "\n")

# ======== SENSOR-KLASSER ========

# --- MPU6050 GYRO ---
class MPU6050:
    def __init__(self):
        import smbus
        self.MPU6050_ADDR = 0x68
        self.bus = smbus.SMBus(1)
        self.offset_z = 0
        self.setup_mpu6050()

    def setup_mpu6050(self):
        self.bus.write_byte_data(self.MPU6050_ADDR, 0x6B, 0x00)
        time.sleep(0.1)
        if os.path.exists("gyro_offset.txt"):
            with open("gyro_offset.txt", "r") as f:
                self.offset_z = float(f.readline().strip())
                print(f"Lastet gyro offset: {self.offset_z:.5f}")

    def read_gyro_z_raw(self):
        high = self.bus.read_byte_data(self.MPU6050_ADDR, 0x47)
        low = self.bus.read_byte_data(self.MPU6050_ADDR, 0x48)
        value = (high << 8) | low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return (value / 131.0)

    def read_gyro_z(self):
        value = self.read_gyro_z_raw()
        return value - self.offset_z

# --- QMC5883L KOMPASS ---
class Kompass:
    def __init__(self):
        import smbus
        self.QMC5883L_ADDRESS = 0x0d
        self.bus = smbus.SMBus(1)
        self.QMC5883L_CTRL1 = 0x09
        self.QMC5883L_SET_RESET = 0x0B
        self.QMC5883L_DATA = 0x00
        self.offset_x = 0
        self.offset_y = 0
        self.setup_compass()

    def setup_compass(self):
        try:
            self.bus.write_byte_data(self.QMC5883L_ADDRESS, self.QMC5883L_SET_RESET, 0x01)
            time.sleep(0.1)
            self.bus.write_byte_data(self.QMC5883L_ADDRESS, self.QMC5883L_CTRL1, 0b00011101)
            time.sleep(0.1)
            if os.path.exists("kompas_offset.txt"):
                with open("kompas_offset.txt", "r") as f:
                    lines = f.readlines()
                    if len(lines) >= 2:
                        self.offset_x = float(lines[0].strip())
                        self.offset_y = float(lines[1].strip())
                        print(f"Offset lastet: x = {self.offset_x:.2f}, y = {self.offset_y:.2f}")
        except Exception as e:
            print(f"Feil ved initiering av kompass: {e}")

    def read_heading(self):
        try:
            data = self.bus.read_i2c_block_data(self.QMC5883L_ADDRESS, self.QMC5883L_DATA, 6)
            x_raw = (data[1] << 8) | data[0]
            y_raw = (data[3] << 8) | data[2]
            x = x_raw - 65536 if x_raw > 32767 else x_raw
            y = y_raw - 65536 if y_raw > 32767 else y_raw
            x -= self.offset_x
            y -= self.offset_y
            heading_rad = math.atan2(y, x)
            heading_deg = math.degrees(heading_rad)
            if heading_deg < 0:
                heading_deg += 360
            return heading_deg
        except Exception as e:
            print(f"Feil ved lesing av kompass: {e}")
            return -1

# --- LIDAR (RPLidar) ---
class Lidar:
    def __init__(self):
        from rplidar import RPLidar
        self.PORT_NAME = "/dev/rplidar"
        self.ROBOT_RADIUS = 117.5
        self.STOP_DISTANCE_LIDAR = 250
        self.current_distance_lidar = 9999
        self.lidar_buffer = []
        self.scan_data = []
        self.running = False
        self.lidar = RPLidar(self.PORT_NAME, baudrate=115200)

    def start(self):
        self.running = True
        threading.Thread(target=self._lidar_thread, daemon=True).start()

    def stop(self):
        self.running = False
        if self.lidar:
            self.lidar.stop()
            self.lidar.disconnect()

    def _lidar_thread(self):
        for scan in self.lidar.iter_scans():
            temp_scan = []
            for measurement in scan:
                angle = measurement[1]
                distance = measurement[2]
                temp_scan.append((angle, distance))
            self.scan_data = temp_scan
            distances = [d for a, d in temp_scan if abs(a - 0) <= 30 or abs(a - 360) <= 30]
            if distances:
                min_distance = min(distances)
                self.current_distance_lidar = min_distance
                self.lidar_buffer.append(min_distance)
                if len(self.lidar_buffer) > 5:
                    self.lidar_buffer.pop(0)
            if not self.running:
                break

    def get_median_lidar_reading(self):
        if not self.lidar_buffer:
            return float('inf')
        sorted_buffer = sorted(self.lidar_buffer)
        n = len(sorted_buffer)
        if n % 2 == 1:
            return sorted_buffer[n // 2]
        else:
            return (sorted_buffer[n // 2 - 1] + sorted_buffer[n // 2]) / 2

    def is_path_clear(self):
        stable_distance = self.get_median_lidar_reading()
        adjusted_distance = stable_distance - self.ROBOT_RADIUS
        return adjusted_distance > self.STOP_DISTANCE_LIDAR

# --- ULTRALYD ---
class Ultrasound:
    def __init__(self):
        import RPi.GPIO as GPIO
        self.GPIO = GPIO
        self.trig_pins = [9, 7, 23, 10]
        self.echo_pins = [8, 6, 24, 11]
        self.sensor_names = ["front_left", "front_right", "back_left", "back_right"]
        self.sensor_angles = {
            "front_left": 340, "front_right": 20,
            "back_left": 222, "back_right": 138
        }
        self.sensor_distances = {name: -1 for name in self.sensor_names}
        self.sensors = dict(zip(self.sensor_names, zip(self.trig_pins, self.echo_pins)))
        self.setup_ultrasound()

    def setup_ultrasound(self):
        self.GPIO.setmode(self.GPIO.BCM)
        for trig in self.trig_pins:
            self.GPIO.setup(trig, self.GPIO.OUT)
            self.GPIO.output(trig, False)
        for echo in self.echo_pins:
            self.GPIO.setup(echo, self.GPIO.IN)
        print("Ultralydsensorene er satt opp. Vent 2 sekunder for stabilisering...")
        time.sleep(2)

    def read_distance(self, trig, echo):
        self.GPIO.output(trig, True)
        time.sleep(0.00001)
        self.GPIO.output(trig, False)
        timeout_start = time.time()
        timeout = 0.02
        pulse_start = pulse_end = None
        while self.GPIO.input(echo) == 0:
            pulse_start = time.time()
            if pulse_start - timeout_start > timeout:
                return -1
        timeout_start = time.time()
        while self.GPIO.input(echo) == 1:
            pulse_end = time.time()
            if pulse_end - timeout_start > timeout:
                return -1
        if pulse_start is None or pulse_end is None:
            return -1
        pulse_duration = (pulse_end - pulse_start) * 1e6
        distance = pulse_duration * 0.0343 / 2
        return distance

    def update_ultrasound_readings(self):
        for sensor, (trig, echo) in self.sensors.items():
            distance = self.read_distance(trig, echo)
            self.sensor_distances[sensor] = distance if distance > 0 else 0

# ======== HEADING FUSION ========
class HeadingTracker:
    def __init__(self, kompass, mpu):
        self.kompass = kompass
        self.mpu = mpu
        self.KOMPASS_JUSTERING = 270
        self.GYRO_VEKT_HOY = 0.98
        self.GYRO_VEKT_LAV = 0.90
        self.ROTASJON_TERSKEL = 30
        self.KOMPASS_CORRECT_INTERVAL = 0.20
        self.last_time = time.time()
        self.last_compass_update = 0
        self.fused_heading = 0
        self.latest_compass = 0
        self.latest_gyro = 0
        self.initialized = False
        self.setup()

    def setup(self):
        heading = self.kompass.read_heading()
        if heading != -1:
            self.fused_heading = (heading + self.KOMPASS_JUSTERING) % 360
            self.latest_compass = self.fused_heading
        else:
            self.fused_heading = 0
            self.latest_compass = 0
        self.initialized = True
        skriv_logg(f"[HEADING] Init: heading={self.fused_heading:.1f}°")

    def update(self):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0 or dt > 1:
            dt = 0.02
        self.last_time = current_time

        gyro_z = self.mpu.read_gyro_z()
        self.latest_gyro = gyro_z
        delta_heading = gyro_z * dt
        self.fused_heading = (self.fused_heading + delta_heading) % 360

        if (current_time - self.last_compass_update) > self.KOMPASS_CORRECT_INTERVAL:
            compass_heading = self.kompass.read_heading()
            if compass_heading != -1:
                compass_heading = (compass_heading + self.KOMPASS_JUSTERING) % 360
                self.latest_compass = compass_heading
                gyro_rate = abs(gyro_z)
                if gyro_rate > self.ROTASJON_TERSKEL:
                    gyro_vekt = self.GYRO_VEKT_HOY
                else:
                    gyro_vekt = self.GYRO_VEKT_LAV
                kompass_vekt = 1 - gyro_vekt
                delta = ((compass_heading - self.fused_heading + 540) % 360) - 180
                correction = delta * kompass_vekt
                self.fused_heading = (self.fused_heading + correction) % 360
                skriv_logg(
                    f"[HEADING] Kompass={compass_heading:.1f}° GyroZ={gyro_z:.2f}°/s Δ={delta:.1f}° Korrigerer {correction:.2f}° → {self.fused_heading:.1f}° (gyro_vekt={gyro_vekt:.2f})"
                )
            else:
                skriv_logg("[HEADING] Kompassfeil, ingen korreksjon!")
            self.last_compass_update = current_time
        return self.fused_heading

    def get_heading(self):
        return self.fused_heading

    def get_compass(self):
        return self.latest_compass

    def get_gyro(self):
        return self.latest_gyro

# ======== MOTORSTYRING ========
class MotorController:
    def __init__(self, port="/dev/esp32", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)

    def send(self, x, y, omega):
        msg = f"{x} {y} {omega:.2f}\n"
        self.ser.write(msg.encode())
        skriv_logg(f"[MOTOR] Sendt: {msg.strip()}")

    def close(self):
        self.ser.close()

# ======== HINDRINGSLOGIKK (Obstacle Avoidance) ========
def normalize_angle(angle):
    return (angle + 180) % 360 - 180

def heading_correction(current_heading, target_heading, k=0.5, max_omega=50.0):
    delta = normalize_angle(target_heading - current_heading)
    omega = k * delta
    return max(-max_omega, min(max_omega, omega))

def finn_aapninger(lidar, sikkerhetsradius):
    fri = [False] * 360
    for angle, distance in lidar.scan_data:
        if not isinstance(angle, (int, float)) or distance <= sikkerhetsradius:
            continue
        delta = math.degrees(math.asin(min(sikkerhetsradius / distance, 1.0)))
        start = int(math.floor(angle - delta)) % 360
        end = int(math.ceil(angle + delta)) % 360
        i = start
        while True:
            fri[i] = True
            if i == end:
                break
            i = (i + 1) % 360
    segments = []
    i = 0
    while i < 360:
        if fri[i]:
            start = i
            while fri[i % 360]:
                i += 1
            end = (i - 1) % 360
            width = (end - start + 1) if end >= start else (360 - start + end + 1)
            segments.append((start, end, width))
        else:
            i += 1
    # wraparound merge
    if segments and segments[0][0] == 0 and segments[-1][1] == 359:
        first = segments.pop(0)
        last = segments.pop(-1)
        merged = (last[0], first[1], first[2] + last[2])
        segments.insert(0, merged)
    return segments

def finn_storste_aapning(lidar, sikkerhetsradius):
    openings = finn_aapninger(lidar, sikkerhetsradius)
    if not openings:
        return None
    best = max(openings, key=lambda x: x[2])
    start, end, width = best
    center = (start + width / 2) % 360
    return center

def ultralyd_blokkert(ultrasound, threshold=10.0):
    for sensor, dist in ultrasound.sensor_distances.items():
        if 0 < dist < threshold:
            skriv_logg(f"[ULTRALYD] Sensor {sensor} blokkert: {dist:.1f} cm")
            return True
    return False

# ======== VISUALISERING (minimal for test/demo) ========
def polar_to_cartesian(angle_deg, distance_cm, heading_deg=0, scale=2.0):
    corrected_angle = -(angle_deg + heading_deg)
    angle_rad = math.radians(corrected_angle)
    x = math.cos(angle_rad) * distance_cm * scale
    y = math.sin(angle_rad) * distance_cm * scale
    return int(300 + x), int(300 - y)

# ======== MAIN LOOP (Pygame) ========
def main():
    STEP = 100
    ROTATE_STEP_DEFAULT = 50.0
    DRIVE_TIME = 2.0
    HEADING_TOLERANCE = 5.0
    ROBOT_DIAMETER = 23.5
    SAFETY_MARGIN = 2.0
    SIKKERHETS_RADIUS = ROBOT_DIAMETER / 2 + SAFETY_MARGIN

    mpu = MPU6050()
    kompass = Kompass()
    lidar = Lidar()
    ultrasound = Ultrasound()
    motor = MotorController()
    heading_tracker = HeadingTracker(kompass, mpu)

    lidar.start()
    state = "IDLE"
    drive_start_time = None
    current_target_angle = None

    pygame.init()
    screen = pygame.display.set_mode((600, 600))
    pygame.display.set_caption("Robotkontroll")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 24)
    running = True

    prev_command = (0, 0, 0)
    while running:
        screen.fill((0, 0, 0))
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        ultrasound.update_ultrasound_readings()
        fused_heading = heading_tracker.update()
        x = y = omega = 0

        if state == "IDLE":
            if ultralyd_blokkert(ultrasound) or not lidar.is_path_clear():
                state = "SEARCH"
                skriv_logg("[HINDRING] Hindring oppdaget, starter søk.")
                x, y, omega = (0, 0, ROTATE_STEP_DEFAULT)
            else:
                x, y, omega = (STEP, 0, 0)
        elif state == "SEARCH":
            angle = finn_storste_aapning(lidar, SIKKERHETS_RADIUS)
            if angle is None:
                x, y, omega = (0, 0, ROTATE_STEP_DEFAULT)
            else:
                current_target_angle = angle
                delta = normalize_angle(current_target_angle - fused_heading)
                if abs(delta) > HEADING_TOLERANCE:
                    omega = heading_correction(fused_heading, current_target_angle)
                    x, y = 0, 0
                else:
                    state = "DRIVING"
                    drive_start_time = time.time()
                    x, y, omega = (STEP, 0, 0)
        elif state == "DRIVING":
            if ultralyd_blokkert(ultrasound):
                state = "IDLE"
                x, y, omega = (0, 0, 0)
            elif (time.time() - drive_start_time) < DRIVE_TIME:
                x, y, omega = (STEP, 0, 0)
            else:
                state = "IDLE"
                x, y, omega = (0, 0, 0)
        current_command = (x, y, omega)
        if current_command != prev_command:
            motor.send(x, y, omega)
            prev_command = current_command

        pygame.draw.circle(screen, (0, 255, 0), (300, 300), 5)
        heading_rad = math.radians(-fused_heading)
        end_x = int(300 + math.cos(heading_rad) * 50)
        end_y = int(300 - math.sin(heading_rad) * 50)
        pygame.draw.line(screen, (0, 0, 255), (300, 300), (end_x, end_y), 4)
        for angle, distance in lidar.scan_data:
            if distance > 0:
                x_vis, y_vis = polar_to_cartesian(angle, distance / 10.0, fused_heading)
                pygame.draw.circle(screen, (255, 255, 255), (x_vis, y_vis), 2)
        pygame.display.update()
        clock.tick(20)

    motor.send(0, 0, 0)
    lidar.stop()
    motor.close()
    pygame.quit()

if __name__ == "__main__":
    main()
