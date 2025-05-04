import lgpio as GPIO
import gpiod
import time

# Set pins
TRIG = 5  # Associate pin 5 to TRIG
ECHO = 6  # Associate pin 6 to ECHO

chip = gpiod.Chip('/dev/gpiochip0')

# set right motor from behind robot
in1_line = chip.get_line(17)
in2_line = chip.get_line(27)
motor1_enableA = chip.get_line(4)
in1_line.request(consumer='motor_control', type=gpiod.LINE_REQ_DIR_OUT)
in2_line.request(consumer='motor_control', type=gpiod.LINE_REQ_DIR_OUT)
motor1_enableA.request(consumer='motor_control', type=gpiod.LINE_REQ_DIR_OUT)
motor1_enableA.set_value(1)

#set left motor from behind robot
in3_line = chip.get_line(23)
in4_line = chip.get_line(24)
motor1_enableB = chip.get_line(26)
in3_line.request(consumer='motor_control', type=gpiod.LINE_REQ_DIR_OUT)
in4_line.request(consumer='motor_control', type=gpiod.LINE_REQ_DIR_OUT)
motor1_enableB.request(consumer='motor_control', type=gpiod.LINE_REQ_DIR_OUT)
motor1_enableB.set_value(1)

def motor_right_forward():
    in1_line.set_value(1)
    in2_line.set_value(0)

def motor_left_forward():
    in3_line.set_value(1)
    in4_line.set_value(0)

def motor_right_backward():
    in1_line.set_value(0)
    in2_line.set_value(1)

def motor_left_backward():
    in3_line.set_value(0)
    in4_line.set_value(1)

def motor_stop():
    in1_line.set_value(0)
    in2_line.set_value(0)
    in3_line.set_value(0)
    in4_line.set_value(0)

# Open the GPIO chip and set the GPIO direction
h = GPIO.gpiochip_open(0)
GPIO.gpio_claim_output(h, TRIG)
GPIO.gpio_claim_input(h, ECHO)

def get_distance():
    # Set TRIG LOW
    GPIO.gpio_write(h, TRIG, 0)
    time.sleep(2)

    # Send 10us pulse to TRIG
    GPIO.gpio_write(h, TRIG, 1)
    time.sleep(0.00001)
    GPIO.gpio_write(h, TRIG, 0)

    # Start recording the time when the wave is sent
    while GPIO.gpio_read(h, ECHO) == 0:
        pulse_start = time.time()

    # Record time of arrival
    while GPIO.gpio_read(h, ECHO) == 1:
        pulse_end = time.time()

    # Calculate the difference in times
    pulse_duration = pulse_end - pulse_start

    # Multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = pulse_duration * 17150
    distance = round(distance, 2)

    return distance/2.54


try:
    while True:
        char = input("Enter 'a' for forward, 'z' for backward, 'x' to stop: ")
        if char == 'a':
            motor_right_forward()
            motor_left_forward()
        elif char == 'z':
            motor_right_backward()
            motor_left_backward()
        elif char == 'x':
            motor_stop()
        
        dist = get_distance()
        print("Measured Distance = {:.2f} in".format(dist))
        time.sleep(1)

except KeyboardInterrupt:
    motor_stop()
    motor1_enableA.set_value(0)
    motor1_enableB.set_value(0)
    GPIO.gpiochip_close(h)
    time.sleep(1)
    print("Program stopped.")

finally:
    in1_line.release()
    in2_line.release()
    motor1_enableA.release()
    in3_line.release()
    in4_line.release()
    motor1_enableB.release()

