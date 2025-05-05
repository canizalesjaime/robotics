import lgpio as GPIO
import gpiod
import time
import threading
import sys

# Set pins
TRIG = 5
ECHO = 6

chip = gpiod.Chip('/dev/gpiochip0')

# Set motor pins
in1_line = chip.get_line(17)
in2_line = chip.get_line(27)
motor1_enableA = chip.get_line(4)
in3_line = chip.get_line(23)
in4_line = chip.get_line(24)
motor1_enableB = chip.get_line(26)

# Request output lines
for line in [in1_line, in2_line, motor1_enableA, in3_line, in4_line, motor1_enableB]:
    line.request(consumer='motor_control', type=gpiod.LINE_REQ_DIR_OUT)
motor1_enableA.set_value(1)
motor1_enableB.set_value(1)

# Motor control functions
def motor_right_forward(): in1_line.set_value(1); in2_line.set_value(0)
def motor_left_forward(): in3_line.set_value(1); in4_line.set_value(0)
def motor_right_backward(): in1_line.set_value(0); in2_line.set_value(1)
def motor_left_backward(): in3_line.set_value(0); in4_line.set_value(1)
def motor_stop():
    in1_line.set_value(0); in2_line.set_value(0)
    in3_line.set_value(0); in4_line.set_value(0)

# Setup GPIO for ultrasonic
h = GPIO.gpiochip_open(0)
GPIO.gpio_claim_output(h, TRIG)
GPIO.gpio_claim_input(h, ECHO)

def get_distance():
    GPIO.gpio_write(h, TRIG, 0)
    time.sleep(0.0002)
    GPIO.gpio_write(h, TRIG, 1)
    time.sleep(0.00001)
    GPIO.gpio_write(h, TRIG, 0)

    start = time.time()
    while GPIO.gpio_read(h, ECHO) == 0:
        start = time.time()
    while GPIO.gpio_read(h, ECHO) == 1:
        end = time.time()

    duration = end - start
    distance = duration * 17150  # cm
    return round(distance / 2.54, 2)  # return inches

# Shared state
current_action = None
action_lock = threading.Lock()

def distance_monitor():
    global current_action
    while True:
        dist = get_distance()
        print(f"Distance: {dist:.2f} in")
        with action_lock:
            if current_action == 'forward' and dist <= 2.0:
                print("Obstacle detected! Stopping.")
                motor_stop()
                current_action = None
        time.sleep(0.2)

# Start distance monitoring thread
monitor_thread = threading.Thread(target=distance_monitor, daemon=True)
monitor_thread.start()

try:
    print("Controls: 'a' = forward, 'z' = backward, 'x' = stop, Ctrl+C to exit.")
    while True:
        char = input("Command: ").lower()
        with action_lock:
            if char == 'a':
                motor_right_forward()
                motor_left_forward()
                current_action = 'forward'
            elif char == 'z':
                motor_right_backward()
                motor_left_backward()
                current_action = 'backward'
            elif char == 'x':
                motor_stop()
                current_action = None
            else:
                print("Invalid command.")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nExiting...")
    motor_stop()
    motor1_enableA.set_value(0)
    motor1_enableB.set_value(0)
    GPIO.gpiochip_close(h)

finally:
    in1_line.release()
    in2_line.release()
    motor1_enableA.release()
    in3_line.release()
    in4_line.release()
    motor1_enableB.release()
