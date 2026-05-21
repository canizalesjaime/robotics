# PWM Calibration 
# Wide pulse range for near-180° travel
# Adjust slightly per servo if needed
# note: since 50HZ, one cycle last 20 ms
# this is # of ticks out of 4096 to represent high
# 4096 comes from the PCA9685’s 12-bit PWM resolution, 
# and 65535 comes from CircuitPython’s 16-bit duty-cycle interface.
# pulses below are estimated, and in general should be calibrated

# STORE PREV ANGLE

import time
import board
import busio
from adafruit_pca9685 import PCA9685
import threading


class BigArmNode():
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50
        self.rotating = False

        self.SERVO_ANGLES = { 0: 90,   # wrist roll MG90S
                              1: 90,   # gripper
                              3: 90,   # wrist pitch
                              4: 90,   # elbow(MG995)
                              5: 90,   # shoulder 
                              7: 90,   # base
        }

        for serv in self.SERVO_ANGLES:
            self.set_servo_angle(serv,90)
            time.sleep(1)


    def set_servo_angle(self,channel, angle):
        MIN_PULSE = 150   # ~0.75 ms, 0 degrees
        MAX_PULSE = 550   # ~2.7 ms, 180 degrees
        min_angle, max_angle = (20,160)
        angle = max(min(angle, max_angle), min_angle)

        pulse = MIN_PULSE + (angle / 180.0) * (MAX_PULSE - MIN_PULSE)
        #pulse = MIN_PULSE + ((angle - min_angle) / (max_angle - min_angle)) * (MAX_PULSE - MIN_PULSE)
        duty = int(pulse / 4096 * 65535)
        self.pca.channels[channel].duty_cycle = duty
        self.SERVO_ANGLES[channel]=angle


    def move_smooth(self,channel, end_angle, step=2, delay=0.02):
        if self.SERVO_ANGLES[channel] < end_angle:
            rng = range(self.SERVO_ANGLES[channel], end_angle + 1, step)
        else:
            rng = range(self.SERVO_ANGLES[channel], end_angle - 1, -step)

        for angle in rng:
            self.set_servo_angle(channel, angle)
            time.sleep(delay)
        #time.sleep(1)


    def straighten_arm(self):
        self.move_smooth(3,170)#shoulder
        self.move_smooth(4,170)#gripper


    def pick_up_static_hacky(self):
        self.move_smooth(3,40)#shoulder
        self.move_smooth(4,0)#gripper


    # def rotate_base(self):
    #     if not self.rotating:
    #         self.rotating = True
    #         threading.Thread(target=self.rotate_loop, daemon=True).start()


    def rotate_loop(self):
        while self.rotating:
            self.move_smooth(0,170)
            self.move_smooth(0,10)
                

    def stop_base(self):
        self.rotating = False
        
    
    def clean_up(self):
        for i,channel in enumerate(self.SERVO_ANGLES):
            self.move_smooth(channel,90)
        self.stop_base()


    def set_angles_api(self, angles):
        for i,channel in enumerate(self.SERVO_ANGLES):
            self.move_smooth(channel,angles[i])

    def set_angle(self, joint, theta):
        self.move_smooth(joint, theta)

def main():
    try:
        node = BigArmNode()
        

        while True:
            joint=int(input("Enter joint: (0-wirst roll, 1-gripper, 3-wrist pitch, 4-elbow, 5-shoulder, 7-base: "))
            angle=int(input("Enter angle(20-160): "))
            node.set_angle(joint,angle)

    finally:
        node.pca.deinit()
        node.clean_up()

main()