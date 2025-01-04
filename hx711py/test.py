import time
import sys
import RPi.GPIO as GPIO
from hx711 import HX711

# Clean up GPIO and exit gracefully
def cleanAndExit():
    print("Cleaning up GPIO...")
    GPIO.cleanup()
    print("Exiting...")
    sys.exit()

# Initialize HX711
hx = HX711(5, 6)  # Pins 5 (DT) and 6 (SCK)

# Set reading format
hx.set_reading_format("MSB", "MSB")

# Reset and tare (set zero point)
hx.reset()
hx.tare()
print("Tare done! Ready to detect objects...")

# Define a threshold for detecting objects
THRESHOLD = 300  # Adjust based on your sensor and setup

# Variables to track object state
object_present = False

try:
    while True:
        # Get the raw weight value
        raw_val = hx.get_weight(5)

        # Detect object presence based on threshold
        if raw_val > THRESHOLD and not object_present:
            print("Object placed.")
            object_present = True
        elif raw_val <= THRESHOLD and object_present:
            print("Object removed.")
            object_present = False

        # Power down and up to reset HX711
        hx.power_down()
        hx.power_up()
        time.sleep(0.1)

except (KeyboardInterrupt, SystemExit):
    cleanAndExit()