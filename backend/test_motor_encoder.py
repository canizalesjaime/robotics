import time
import lgpio as GPIO


# ---------------------------------------------------------
# GPIO setup
# ---------------------------------------------------------

h = GPIO.gpiochip_open(0)

# TB6612 motor control
AIN1 = 17
AIN2 = 27

BIN1 = 23
BIN2 = 24

PWMA = 12
PWMB = 13

STBY = 25

# Encoder pins
LEFT_A = 5
LEFT_B = 6

RIGHT_A = 16
RIGHT_B = 26


# ---------------------------------------------------------
# Motor GPIO setup
# ---------------------------------------------------------

GPIO.gpio_claim_output(h, AIN1)
GPIO.gpio_claim_output(h, AIN2)

GPIO.gpio_claim_output(h, BIN1)
GPIO.gpio_claim_output(h, BIN2)

GPIO.gpio_claim_output(h, STBY)

GPIO.gpio_claim_output(h, PWMA)
GPIO.gpio_claim_output(h, PWMB)

# Enable TB6612
GPIO.gpio_write(h, STBY, 1)


# ---------------------------------------------------------
# Encoder GPIO setup
# ---------------------------------------------------------

GPIO.gpio_claim_input(h, LEFT_A)
GPIO.gpio_claim_input(h, LEFT_B)

GPIO.gpio_claim_input(h, RIGHT_A)
GPIO.gpio_claim_input(h, RIGHT_B)


left_ticks = 0
right_ticks = 0


# ---------------------------------------------------------
# Encoder callbacks
# ---------------------------------------------------------

def left_encoder_event(chip, gpio, level, timestamp):
    global left_ticks

    if GPIO.gpio_read(h, LEFT_B) == 0:
        left_ticks += 1
    else:
        left_ticks -= 1

    print("Left:", left_ticks, " Right:", right_ticks)


def right_encoder_event(chip, gpio, level, timestamp):
    global right_ticks

    if GPIO.gpio_read(h, RIGHT_B) == 0:
        right_ticks += 1
    else:
        right_ticks -= 1

    print("Left:", left_ticks, " Right:", right_ticks)


left_callback = GPIO.callback(
    h,
    LEFT_A,
    GPIO.RISING_EDGE,
    left_encoder_event
)

right_callback = GPIO.callback(
    h,
    RIGHT_A,
    GPIO.RISING_EDGE,
    right_encoder_event
)


# ---------------------------------------------------------
# Motor functions
# ---------------------------------------------------------

def forward(speed):

    # Left motor
    GPIO.gpio_write(h, AIN1, 0)
    GPIO.gpio_write(h, AIN2, 1)

    # Right motor
    GPIO.gpio_write(h, BIN1, 1)
    GPIO.gpio_write(h, BIN2, 0)

    # PWM
    GPIO.tx_pwm(h, PWMA, 1000, speed)
    GPIO.tx_pwm(h, PWMB, 1000, speed)


def stop():

    GPIO.tx_pwm(h, PWMA, 1000, 0)
    GPIO.tx_pwm(h, PWMB, 1000, 0)

    GPIO.gpio_write(h, AIN1, 0)
    GPIO.gpio_write(h, AIN2, 0)

    GPIO.gpio_write(h, BIN1, 0)
    GPIO.gpio_write(h, BIN2, 0)


# ---------------------------------------------------------
# Test
# ---------------------------------------------------------

try:

    print("Starting motor test...")
    print("Ticks:", left_ticks, right_ticks)

    # Run motors at 30% speed
    forward(30)

    # Run for 5 seconds
    start = time.monotonic()

    while time.monotonic() - start < 5:

        print(
            "Ticks:",
            left_ticks,
            right_ticks
        )

        time.sleep(0.5)

    stop()

    print()
    print("FINAL TICKS:")
    print("Left :", left_ticks)
    print("Right:", right_ticks)


finally:

    stop()

    left_callback.cancel()
    right_callback.cancel()

    GPIO.gpio_write(h, STBY, 0)

    GPIO.gpiochip_close(h)