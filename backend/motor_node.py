# PWM is used to control motor speed. Duty cycle(speed) is the percentage of 
# time high per cycle. If high -> motor on if low motor -> off.
# The more time the the signal is high during a cycle the faster the speed.

import gpiod
import lgpio as GPIO


class MotorNode():
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


    ###########################################################################
    def move(self,cmd):
        print(cmd, "speed: ", self.curr_speed)

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
    def release_lines(self):
        for name, line in self.drivers_input_map.items():
            line.set_value(0)
            line.release()
        
        for name, pin in self.drivers_enable_pin_map.items():
            GPIO.tx_pwm(self.h, pin, self.frequency, 0)
        GPIO.gpiochip_close(self.h)

        self.stby.set_value(0)
        self.stby.release()


    ###########################################################################
    def set_speed(self,percent):
        percent=min(max(percent,0),100)
        
        for name, pin in self.drivers_enable_pin_map.items():
            GPIO.tx_pwm(self.h, pin, self.frequency, percent)
            

###############################################################################
def main():
    try:
        motor=MotorNode()
        while True:
            cmd=input("enter one of the following - f(forward), b(back), i(increase), d(decrease), rotate_left(rl), rotate_right(rr): ")
            motor.move(cmd)
        
    finally:
        motor.release_lines()


#main()
