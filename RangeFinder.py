import serial
import time

# Åbn seriel forbindelse til sensor (UART)
ser = serial.Serial("/dev/serial0", 9600, timeout=1)  # Bruger UART på RPi
time.sleep(2)  # Vent på initialisering

try:
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').strip()  # Læs og dekod
            if data.isdigit():
                distance = int(data)  # Konverter til integer (mm)
                print(f"Afstand: {distance} mm")
except KeyboardInterrupt:
    print("Afslutter program...")
finally:
    ser.close()
