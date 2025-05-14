import lgpio as GPIO
import time

# Set up the GPIO chip and pins
TRIG = 5   # GPIO5
ECHO = 6   # GPIO6

h = GPIO.gpiochip_open(0)  # Open /dev/gpiochip0

# Set TRIG as output, ECHO as input
GPIO.gpio_claim_output(h, TRIG)
GPIO.gpio_claim_input(h, ECHO)

try:
    while True:
        # Ensure trigger is low
        GPIO.gpio_write(h, TRIG, 0)
        time.sleep(0.5)

        # Send 10µs HIGH pulse on TRIG
        GPIO.gpio_write(h, TRIG, 1)
        time.sleep(10e-6)  # 10 microseconds
        GPIO.gpio_write(h, TRIG, 0)

        # Wait ~10 ms so ECHO will go high
        time.sleep(0.01)

        print("Pulse sent. Measure ECHO voltage now.")
        time.sleep(10)  # Time to measure with multimeter

except KeyboardInterrupt:
    print("Exiting")

finally:
    GPIO.gpiochip_close(h)
