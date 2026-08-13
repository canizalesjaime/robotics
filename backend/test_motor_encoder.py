import time
import lgpio as GPIO


# ============================================================
# GPIO
# ============================================================

h = GPIO.gpiochip_open(0)


# ------------------------------------------------------------
# TB6612
# ------------------------------------------------------------

# Left motor
LEFT_IN1 = 17
LEFT_IN2 = 27
LEFT_PWM = 12

# Right motor
RIGHT_IN1 = 23
RIGHT_IN2 = 24
RIGHT_PWM = 13

STBY = 25


# ------------------------------------------------------------
# Encoders
# ------------------------------------------------------------

LEFT_A = 5
LEFT_B = 6

RIGHT_A = 16
RIGHT_B = 26


# ============================================================
# Motor GPIO setup
# ============================================================

GPIO.gpio_claim_output(h, LEFT_IN1)
GPIO.gpio_claim_output(h, LEFT_IN2)
GPIO.gpio_claim_output(h, LEFT_PWM)

GPIO.gpio_claim_output(h, RIGHT_IN1)
GPIO.gpio_claim_output(h, RIGHT_IN2)
GPIO.gpio_claim_output(h, RIGHT_PWM)

GPIO.gpio_claim_output(h, STBY)

GPIO.gpio_write(h, STBY, 1)


# ============================================================
# Encoder GPIO setup
# ============================================================

GPIO.gpio_claim_input(h, LEFT_A)
GPIO.gpio_claim_input(h, LEFT_B)

GPIO.gpio_claim_input(h, RIGHT_A)
GPIO.gpio_claim_input(h, RIGHT_B)


# ============================================================
# Tick counters
# ============================================================

left_ticks = 0
right_ticks = 0


# ============================================================
# Encoder callbacks
# ============================================================

def left_encoder_callback(chip, gpio, level, timestamp):

    global left_ticks

    print("LEFT CALLBACK")

    if GPIO.gpio_read(h, LEFT_B) == 0:
        left_ticks += 1
    else:
        left_ticks -= 1

    print("Left ticks:", left_ticks)


def right_encoder_callback(chip, gpio, level, timestamp):

    global right_ticks

    print("RIGHT CALLBACK")

    if GPIO.gpio_read(h, RIGHT_B) == 0:
        right_ticks += 1
    else:
        right_ticks -= 1

    print("Right ticks:", right_ticks)


left_callback = GPIO.callback(
    h,
    LEFT_A,
    GPIO.RISING_EDGE,
    left_encoder_callback
)

right_callback = GPIO.callback(
    h,
    RIGHT_A,
    GPIO.RISING_EDGE,
    right_encoder_callback
)


# ============================================================
# Motor control
# ============================================================

def left_forward(speed):

    GPIO.gpio_write(h, LEFT_IN1, 0)
    GPIO.gpio_write(h, LEFT_IN2, 1)

    GPIO.tx_pwm(
        h,
        LEFT_PWM,
        1000,
        speed
    )


def stop():

    GPIO.tx_pwm(h, LEFT_PWM, 1000, 0)
    GPIO.tx_pwm(h, RIGHT_PWM, 1000, 0)

    GPIO.gpio_write(h, LEFT_IN1, 0)
    GPIO.gpio_write(h, LEFT_IN2, 0)

    GPIO.gpio_write(h, RIGHT_IN1, 0)
    GPIO.gpio_write(h, RIGHT_IN2, 0)


# ============================================================
# Test
# ============================================================

try:

    print("Starting LEFT motor test...")
    print("Initial LEFT A:",
          GPIO.gpio_read(h, LEFT_A))

    # Only move the LEFT motor
    left_forward(30)

    start = time.monotonic()

    while time.monotonic() - start < 5:

        print(
            "GPIO 5:",
            GPIO.gpio_read(h, LEFT_A),
            " | Left ticks:",
            left_ticks,
            " | Right ticks:",
            right_ticks
        )

        time.sleep(0.5)

    stop()

    print()
    print("================================")
    print("FINAL LEFT TICKS :", left_ticks)
    print("FINAL RIGHT TICKS:", right_ticks)
    print("================================")


finally:

    stop()

    left_callback.cancel()
    right_callback.cancel()

    GPIO.gpio_write(h, STBY, 0)

    GPIO.gpiochip_close(h)