import serial 
import time

# Adjust port to your Arduino's port
arduino = serial.Serial(port='COM7', baudrate=115200, timeout=.1)

while True:
    data = arduino.readline()
    print(data)