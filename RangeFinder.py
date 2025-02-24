# coding: utf-8
import time
import RPi.GPIO as GPIO

# Brug BCM GPIO-numre
GPIO.setmode(GPIO.BCM)

# Definer GPIO-pins
GPIO_TRIGECHO = 15  # Til ultralydssensoren
GPIO_LED = 17       # Til LED'en

print("Ultrasonic Measurement with LED Indicator")

# Konfigurer pins
GPIO.setup(GPIO_TRIGECHO, GPIO.OUT)  # Output til trigger
GPIO.setup(GPIO_LED, GPIO.OUT)       # Output til LED
GPIO.output(GPIO_TRIGECHO, False)    # Sørg for, at trigger starter lavt

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

# Variabler til LED-blink timing
last_distance_time = time.time()
last_led_toggle = time.time()
led_state = False
blink_interval = 0

try:
    while True:
        current_time = time.time()

        # Mål og udskriv afstand hver 1 sekund
        if current_time - last_distance_time >= 1.0:
            distance = measure()
            print("  Distance : %.1f cm" % distance)
            last_distance_time = current_time  # Opdater sidste måletidspunkt

            # Opdater LED-blink-interval
            if 25 <= distance <= 30:
                blink_interval = 2.0  # Blink hver 2. sekund
            elif 18 <= distance < 25:
                blink_interval = 1.0  # Blink hver 1. sekund
            elif distance < 18:
                GPIO.output(GPIO_LED, True)  # Konstant tændt
                blink_interval = 0  # Stop blink
            else:
                GPIO.output(GPIO_LED, False)  # LED slukket
                blink_interval = 0

        # Håndter LED-blink uafhængigt af måling
        if blink_interval > 0 and current_time - last_led_toggle >= blink_interval / 2:
            led_state = not led_state  # Skift LED-tilstand
            GPIO.output(GPIO_LED, led_state)
            last_led_toggle = current_time  # Opdater sidste blink-tidspunkt

        time.sleep(0.05)  # Kort pause for at aflaste CPU'en

except KeyboardInterrupt:
    print("Stop")
    GPIO.cleanup()
