# coding: utf-8
import time
import RPi.GPIO as GPIO

# Brug BCM GPIO-numre
GPIO.setmode(GPIO.BCM)

# Definer GPIO-pins
GPIO_LED = 17  # LED sidder på GPIO 17

# Konfigurer LED som output
GPIO.setup(GPIO_LED, GPIO.OUT)

try:
    print("LED tændt! Tryk Ctrl + C for at slukke.")
    GPIO.output(GPIO_LED, True)  # Tænd LED

    while True:
        time.sleep(1)  # Hold programmet kørende

except KeyboardInterrupt:
    print("\nStopper programmet og slukker LED...")
    GPIO.output(GPIO_LED, False)  # Sluk LED
    GPIO.cleanup()  # Rydder op i GPIO-pins
