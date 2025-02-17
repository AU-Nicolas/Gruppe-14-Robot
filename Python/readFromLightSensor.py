import smbus
import time


# Get I2C bus
bus = smbus.SMBus(1) # or smbus.SMBus(0)

# ISL29125 address, 0x44(68)
# Select configuation-1register, 0x01(01)
# 0x0D(13) Operation: RGB, Range: 360 lux, Res: 16 Bits
bus.write_byte_data(0x44, 0x01, 0x05)

time.sleep(1)

print("Reading colour values and displaying them in a new window\n")

green = 0
red = 0
blue = 0

def colorDisplay():
    # Range for red
    if green > red and green > blue: 
        print("GREEN")
    elif blue > red:
        print("BLUE")
    else:
        print("RED")

def getAndUpdateColour():
    global green
    global red
    global blue 
    while True:
    # Read the data from the sensor
        green = bus.read_word_data(0x44, 0x09)
        red = bus.read_word_data(0x44, 0x0B)
        blue = bus.read_word_data(0x44, 0x0D)

        # Output data to the console RGB values
        green = ((green & 0xFF) << 8) | (green >> 8)
        red = ((red & 0xFF) << 8) | (red >> 8)
        blue = ((blue & 0xFF) << 8) | (blue >> 8)
        # Uncomment the line below when you have read the red, green and blue values
        print("RED = ", red, "GREEN = ", green, "BLUE = ", blue)


        time.sleep(2)

getAndUpdateColour()



