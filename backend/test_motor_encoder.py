import time
import lgpio as GPIO

h = GPIO.gpiochip_open(0)

GPIO.gpio_claim_input(h, 5)

last = GPIO.gpio_read(h, 5)

print("Initial:", last)

while True:

    value = GPIO.gpio_read(h, 5)

    if value != last:
        print("GPIO 5 =", value)
        last = value

    time.sleep(0.001)