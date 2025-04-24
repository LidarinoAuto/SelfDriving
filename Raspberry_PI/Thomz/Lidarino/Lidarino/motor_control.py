import serial
import time

ESP_PORT = "/dev/esp32"
esp = serial.Serial(ESP_PORT, 115200, timeout=1)
time.sleep(2)

def send_command(command):
    print(f"Sender: {command.strip()}")
    esp.write(command.encode())

def move_forward():
    send_command("100 0 0\n")

def stop_robot():
    send_command("0 0 0\n")