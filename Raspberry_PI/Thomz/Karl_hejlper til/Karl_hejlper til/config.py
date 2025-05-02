# config.py
"""
Central configuration parameters for the robot system.
"""

# Serial settings for LIDAR
LIDAR_PORT = '/dev/rplidar'
LIDAR_BAUDRATE = 115200
LIDAR_BUFFER_SIZE = 5  # number of scans to keep in rolling buffer

# GPIO pins for ultrasonic sensors
ULTRASOUND_SENSOR_PINS = {
    'front_left_trigger': 17,
    'front_left_echo': 18,
    'front_right_trigger': 22,
    'front_right_echo': 23
}
ULTRASOUND_TIMEOUT_MS = 1000  # max echo wait in milliseconds

# I2C settings for compass
COMPASS_I2C_BUS = 1
COMPASS_ADDRESS = 0x1E
HEADING_THRESHOLD_DEGREES = 5

# I2C settings for gyroscope (e.g., MPU-6050)
GYRO_I2C_BUS = 1
GYRO_ADDRESS = 0x68

# Serial settings for motor control via ESP32
SERIAL_PORT = '/dev/esp32'
SERIAL_BAUDRATE = 115200

# Logging
LOG_FILE = 'robot.log'
LOG_MAX_BYTES = 1_000_000
LOG_BACKUP_COUNT = 5