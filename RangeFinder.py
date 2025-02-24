# coding: utf-8
import time
import RPi.GPIO as GPIO

# Use BCM GPIO references
# instead of physical pin numbers
GPIO.setmode(GPIO.BCM)

# Define GPIO to use on Pi
GPIO_TRIGECHO = 15
GPIO_LED = 17 # LED.

print ("Ultrasonic Measurement")

# Set pins as output and input
GPIO.setup(GPIO_TRIGECHO,GPIO.OUT)  # Initial state as output

GPIO.setup(GPIO_LED, GPIO.OUT)       # Output til LED

# Set trigger to False (Low)
GPIO.output(GPIO_TRIGECHO, False)

def measure():
    """Måler afstand med ultralydssensoren"""
    GPIO.output(GPIO_TRIGECHO, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGECHO, False)
    
    # Skift til input for at modtage echo
    GPIO.setup(GPIO_TRIGECHO, GPIO.IN)
    start = time.time()

    while GPIO.input(GPIO_TRIGECHO) == 0:
        start = time.time()
    
    while GPIO.input(GPIO_TRIGECHO) == 1:
        stop = time.time()
    
    GPIO.setup(GPIO_TRIGECHO, GPIO.OUT)
    GPIO.output(GPIO_TRIGECHO, False)
    
    elapsed = stop - start
    distance = (elapsed * 34300) / 2.0
    return distance

def led_control():
    """Styrer LED-blink afhængigt af den aktuelle afstand"""
    global current_distance
    while True:
        if 25 <= current_distance <= 30:
            GPIO.output(GPIO_LED, True)
            time.sleep(0.5)
            GPIO.output(GPIO_LED, False)
            time.sleep(1.5)  # Total blinkperiode = 2 sek

        elif 18 <= current_distance < 25:
            GPIO.output(GPIO_LED, True)
            time.sleep(0.5)
            GPIO.output(GPIO_LED, False)
            time.sleep(0.5)  # Total blinkperiode = 1 sek

        elif current_distance < 18:
            GPIO.output(GPIO_LED, True)  # Konstant tændt

        else:
            GPIO.output(GPIO_LED, False)  # Slukket hvis uden for range

        time.sleep(0.1)  # Kort pause, så loopet kører jævnt

# Start LED-tråd
led_thread = threading.Thread(target=led_control, daemon=True)
led_thread.start()

try:
    while True:
        current_distance = measure()
        print("  Distance : %.1f cm" % current_distance)
        time.sleep(1)  # Udskriv afstand hvert sekund

except KeyboardInterrupt:
    print("Stop")
    GPIO.cleanup()
