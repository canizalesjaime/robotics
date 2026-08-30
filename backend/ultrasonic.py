import lgpio as GPIO
import time

class Ultrasonic():
    def __init__(self):
        self.TRIG = 5
        self.ECHO = 6
        self.h = GPIO.gpiochip_open(0)
        GPIO.gpio_claim_output(self.h, self.TRIG)
        GPIO.gpio_claim_input(self.h, self.ECHO)


    def get_distance(self):
        GPIO.gpio_write(self.h, self.TRIG, 0)
        time.sleep(0.0002)
        GPIO.gpio_write(self.h, self.TRIG, 1)
        time.sleep(0.00001)
        GPIO.gpio_write(self.h, self.TRIG, 0)

        start = time.time()
        while GPIO.gpio_read(self.h, self.ECHO) == 0:
            start = time.time()
        while GPIO.gpio_read(self.h, self.ECHO) == 1:
            end = time.time()

        duration = end - start
        distance_cm = duration * 17150
        return round(distance_cm / 2.54, 2)  # inches


    def free_gpio(self):
        GPIO.gpiochip_close(self.h)

def main():
    node = Ultrasonic()
    try:
        while(True):
            print(node.get_distance())

    except KeyboardInterrupt:
        print("\nExiting...")
    

    finally:
        node.free_gpio()

if __name__ == '__main__':
    main()
