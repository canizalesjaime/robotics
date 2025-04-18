import gpiod
import time

chip = gpiod.Chip('/dev/gpiochip0')

in1_line = chip.get_line(17)
in2_line = chip.get_line(27)
in3_line = chip.get_line(23)
in4_line = chip.get_line(24)
# motor1_enableA = chip.get_line(4)#PIN NUM
# motor1_enableB = chip.get_line(26)#PIN NUM

in1_line.request(consumer='motor_control', type=gpiod.LINE_REQ_DIR_OUT)
in2_line.request(consumer='motor_control', type=gpiod.LINE_REQ_DIR_OUT)
in3_line.request(consumer='motor_control', type=gpiod.LINE_REQ_DIR_OUT)
in4_line.request(consumer='motor_control', type=gpiod.LINE_REQ_DIR_OUT)
# motor1_enableA.request(consumer='motor_control', type=gpiod.LINE_REQ_DIR_OUT)
# motor1_enableB.request(consumer='motor_control', type=gpiod.LINE_REQ_DIR_OUT)

# motor1_enableA.set_value(1)
# motor1_enableB.set_value(1)

def motor_forward():
    in1_line.set_value(1)
    in3_line.set_value(1)
    in2_line.set_value(0)
    in4_line.set_value(0)

def motor_backward():
    in1_line.set_value(0)
    in3_line.set_value(0)
    in2_line.set_value(1)
    in4_line.set_value(1)

def motor_stop():
    in1_line.set_value(0)
    in2_line.set_value(0)
    in3_line.set_value(0)
    in4_line.set_value(0)
    # motor1_enableA.set_value(0)
    # motor1_enableB.set_value(0)

try:
    while True:
        char = input("Enter 'a' for forward, 'z' for backward, 'x' to stop: ")
        if char == 'a':
            motor_forward()
        elif char == 'z':
            motor_backward()
        elif char == 'x':
            motor_stop()

except KeyboardInterrupt:
    motor_stop()
    time.sleep(1)
    print("Program stopped.")

finally:
    in1_line.release()
    in2_line.release()
    in3_line.release()
    in4_line.release()
    # motor1_enableA.release()
    # motor1_enableB.release()
