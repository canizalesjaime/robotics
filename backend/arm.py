# PWM Calibration 
# Wide pulse range for near-180° travel
# Adjust slightly per servo if needed
# note: since 50HZ, one cycle last 20 ms
# this is # of ticks out of 4096 to represent high
# 4096 comes from the PCA9685’s 12-bit PWM resolution, 
# and 65535 comes from CircuitPython’s 16-bit duty-cycle interface.
# pulses below are estimated, and in general should be calibrated

import time
import board
import busio
from adafruit_pca9685 import PCA9685
#import threading

# def rotate_base(arm):
#     if not arm.rotating:
#         arm.rotating=True
#         threading.Thread(target=arm.rotate_loop, daemon=True).start()

class ArmNode():
    def __init__(self,arm="base"):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50
        self.rotating = False
        self.SERVO_ANGLES={} #{"joint name:[pca channel, current angle]"}

        if arm=="base":
            self.SERVO_ANGLES={"base":[0,90]}
    
        elif arm=="small":
            self.SERVO_ANGLES = {"base":[0, 90], # MG90S below
                                 "elbow": [1, 90],
                                 "shoulder": [3, 90],
                                 "gripper": [4,90] }

        else:
            self.SERVO_ANGLES = {"wrist roll":[0, 90],# MG90S below
                                 "gripper":[1, 90],
                                 "wrist pitch":[3, 90],
                                 "elbow":[4, 90],  # MG995 below
                                 "shoulder":[5, 90],    
                                 "base":[7, 90] }

        for joint, channel_angle in self.SERVO_ANGLES.items():
            channel, angle = channel_angle
            self.set_servo_angle(joint,angle)
            time.sleep(1)


    def set_servo_angle(self,joint, angle):
        channel = self.SERVO_ANGLES[joint][0]
        MIN_PULSE = 150   # ~0.75 ms, 0 degrees
        MAX_PULSE = 550   # ~2.7 ms, 180 degrees
        min_angle, max_angle = (20,160)
        angle = max(min(angle, max_angle), min_angle)

        #pulse = MIN_PULSE + (angle / 180.0) * (MAX_PULSE - MIN_PULSE)
        pulse = MIN_PULSE + ((angle - min_angle) / (max_angle - min_angle)) * (MAX_PULSE - MIN_PULSE)
        duty = int(pulse)<<4
        self.pca.channels[channel].duty_cycle = duty
        self.SERVO_ANGLES[joint][1]=angle


    def move_smooth(self,joint, end_angle, step=2, delay=0.02):
        curr_angle=self.SERVO_ANGLES[joint][1]
        if curr_angle < end_angle:
            rng = range(curr_angle, end_angle + 1, step)
        else:
            rng = range(curr_angle, end_angle - 1, -step)

        for angle in rng:
            self.set_servo_angle(joint, angle)
            time.sleep(delay)


    def rotate_base(self, val=True):
        if not self.rotating:
            self.rotating=val
            while self.rotating:
                self.move_smooth("base",150)
                self.move_smooth("base",30)
                    

    def stop_base(self):
        self.rotating = False
        
    
    def clean_up(self):
        for joint in self.SERVO_ANGLES:
            self.move_smooth(joint,90)  
        self.stop_base()


    def set_angles_api(self, angles):
        for i,joint in enumerate(self.SERVO_ANGLES):
            self.move_smooth(joint,angles[i])

def main():
    try:
        node = ArmNode()
        node.rotate_base()
            

    finally:
        node.clean_up()
        node.pca.deinit()
        

if __name__ == '__main__':
    main()