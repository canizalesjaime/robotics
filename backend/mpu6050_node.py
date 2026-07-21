# mpu6050 GY-521 Module
# right-hand rule. Z is thumb, Y is toward VCC
# gyroscrope computes the change in angular velocity
# accelerometer computes acceleration(rate of change in velocity)

from mpu6050 import mpu6050
import math

class Mpu6050Node():
    def __init__(self):
        self.sensor = mpu6050(0x68)


    def get_roll_pitch(self, accel):
        ax = accel['x']
        ay = accel['y']
        az = accel['z']

        # Convert to roll and pitch (in degrees)
        roll  = math.degrees(math.atan2(ay, az))
        pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))

        return roll, pitch
    
    def sensor_data(self):
        accel = self.sensor.get_accel_data()
        gyro = self.sensor.get_gyro_data()
        temp = self.sensor.get_temp()
        roll, pitch = self.get_roll_pitch(accel)

        return {"roll": roll, "pitch":pitch, "temp":temp, 
                "lin_accel.x": accel['x'] * 9.80665,
                "lin_accel.y": accel['y'] * 9.80665,
                "lin_accel.z": accel['z'] * 9.80665, 
                "ang_vel.x": math.radians(gyro['x']),
                "ang_vel.y": math.radians(gyro['y']),
                "ang_vel.z": math.radians(gyro['z'])
                }
        

def main():
    try:
        node = Mpu6050Node()
        for i in range(20):
            print(node.sensor_data())
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()