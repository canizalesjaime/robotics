# PWM is used to control motor speed. Duty cycle(speed) is the percentage of 
# time high per cycle. If high -> motor on if low motor -> off.
# The more time the the signal is high during a cycle the faster the speed.

import time
import gpiod
import lgpio as GPIO


class MotorNode():
    TICKS_PER_WHEEL_REV = 1080 # 12 PPR × 90:1 gearbox

    def __init__(self):
        self.chip = gpiod.Chip('/dev/gpiochip0')
        # gpios used in drivers_map
        self.drivers_input_map ={"driverA_in1": self.chip.get_line(17), 
                                 "driverA_in2": self.chip.get_line(27),
                                 "driverB_in1": self.chip.get_line(23),
                                 "driverB_in2": self.chip.get_line(24) }
        self.drivers_enable_pin_map ={"driverA_enA":12,"driverB_enB":13}

        self.stby = self.chip.get_line(25)
        self.stby.request(consumer='motor_control',type=gpiod.LINE_REQ_DIR_OUT)
        self.stby.set_value(1)

        for name, line in self.drivers_input_map.items():
            line.request(consumer='motor_control', type=gpiod.LINE_REQ_DIR_OUT)

        self.h = GPIO.gpiochip_open(0)
        for name, pin in self.drivers_enable_pin_map.items():
            GPIO.gpio_claim_output(self.h, pin) 

        self.frequency =1000
        self.curr_speed=30
        self.set_speed(self.curr_speed)

        # Encoder pins
        self.left_encoder_a = 5
        self.left_encoder_b = 6

        self.right_encoder_a = 16
        self.right_encoder_b = 26

        GPIO.gpio_claim_input(self.h, self.left_encoder_a)
        GPIO.gpio_claim_input(self.h, self.left_encoder_b)
        GPIO.gpio_claim_input(self.h, self.right_encoder_a)
        GPIO.gpio_claim_input(self.h, self.right_encoder_b)

        self.left_ticks = 0
        self.right_ticks = 0

        # Values used to calculate velocity
        self.previous_left_ticks = 0
        self.previous_right_ticks = 0
        self.previous_time = time.monotonic()

        # Encoder callbacks
        # Manufacturer's example uses rising edge of A. B determines direction.
        self.left_callback = GPIO.callback(
            self.h,
            self.left_encoder_a,
            GPIO.RISING_EDGE,
            self.left_encoder_event
        )

        self.right_callback = GPIO.callback(
            self.h,
            self.right_encoder_a,
            GPIO.RISING_EDGE,
            self.right_encoder_event
        )

    ###########################################################################
    def move(self,cmd):
        if cmd == 'f':
            self.set_motor([0, 1, 1, 0])
        elif cmd == 'b':
            self.set_motor([1, 0, 0, 1])
        elif cmd == 'rl':
            self.set_motor([1, 0, 1, 0])
        elif cmd == 'rr':
            self.set_motor([0, 1, 0, 1])
        elif cmd == 's':
            self.set_motor([0, 0, 0, 0])
        elif cmd == 'i':
            self.curr_speed=self.curr_speed+5
            self.set_speed(self.curr_speed)
        elif cmd == 'd':
            self.curr_speed=self.curr_speed-5
            self.set_speed(self.curr_speed)
        elif cmd == 'l':
            self.set_motor([0,0,1,0])
        elif cmd == 'r':
            self.set_motor([0,1,0,0])
        else:
            print("error wrong command")

    # python version >= 3.7, and dictionaries are ordered by insert
    ###########################################################################
    def set_motor(self,motor_inputs):
        i = 0 
        for name, line in self.drivers_input_map.items():
            line.set_value(motor_inputs[i])
            i=i+1
        

    ###########################################################################
    def set_speed(self,percent):
        percent=min(max(percent,0),100)
        
        for name, pin in self.drivers_enable_pin_map.items():
            GPIO.tx_pwm(self.h, pin, self.frequency, percent)


    # Encoder callbacks
    ###########################################################################

    def left_encoder_event(self, chip, gpio, level, timestamp):
        if GPIO.gpio_read(self.h, self.left_encoder_b) == 0:
            self.left_ticks += 1
        else:
            self.left_ticks -= 1

    ################################################################
    def right_encoder_event(self, chip, gpio, level, timestamp):
        if GPIO.gpio_read(self.h, self.right_encoder_b) == 0:
            self.right_ticks += 1
        else:
            self.right_ticks -= 1

    ################################################################
    def get_ticks(self):
        return self.left_ticks, self.right_ticks

    ################################################################
    # Calculate wheel angular velocity
    ################################################################
    def get_wheel_velocities(self):
        current_time = time.monotonic()

        current_left_ticks = self.left_ticks
        current_right_ticks = self.right_ticks

        dt = current_time - self.previous_time

        if dt <= 0:
            return 0.0, 0.0

        # Number of ticks since last measurement
        left_delta_ticks = (
            current_left_ticks - self.previous_left_ticks
        )

        right_delta_ticks = (
            current_right_ticks - self.previous_right_ticks
        )

        # Save current values for next measurement
        self.previous_left_ticks = current_left_ticks
        self.previous_right_ticks = current_right_ticks
        self.previous_time = current_time

        # ticks → revolutions
        left_revolutions = (
            left_delta_ticks / self.TICKS_PER_WHEEL_REV
        )

        right_revolutions = (
            right_delta_ticks / self.TICKS_PER_WHEEL_REV
        )

        # revolutions → radians
        left_angle = left_revolutions * 2.0 * 3.14159265359
        right_angle = right_revolutions * 2.0 * 3.14159265359

        # radians → radians/second
        left_velocity = left_angle / dt
        right_velocity = right_angle / dt

        return left_velocity, right_velocity

    ###########################################################################
    def reset_ticks(self):
        self.left_ticks = 0
        self.right_ticks = 0

        self.previous_left_ticks = 0
        self.previous_right_ticks = 0

        self.previous_time = time.monotonic()

    ###########################################################################
    def release_lines(self):
        for name, line in self.drivers_input_map.items():
            line.set_value(0)
            line.release()
        
        for name, pin in self.drivers_enable_pin_map.items():
            GPIO.tx_pwm(self.h, pin, self.frequency, 0)

        self.left_callback.cancel()
        self.right_callback.cancel()

        GPIO.gpiochip_close(self.h)

        self.stby.set_value(0)
        self.stby.release()

###############################################################################
def main():
    try:
        motor=MotorNode()
        for i in range(3):
            cmd=input("enter one of the following - f(forward), b(back), i(increase), d(decrease), rotate_left(rl), rotate_right(rr): ")
            motor.move(cmd)
        
    finally:
        motor.release_lines()


if __name__ == '__main__':
    main()
