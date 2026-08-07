import board
import busio
import adafruit_bno055


class ImuSensor():
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)

    def get_euler(self):
        return self.sensor.euler

    def get_quaternion(self):
        return self.sensor.quaternion

    def get_gyro(self): # angular acceleration
        return self.sensor.gyro

    def get_acceleration(self): # linear acceleration
            return self.sensor.acceleration

    def get_magnetic(self):
            return self.sensor.magnetic

    def get_temperature(self):
            return self.sensor.temperature

    def get_calibration_status(self):
            return self.sensor.calibration_status
        

def main():
    sensor=ImuSensor()    
    while True:
        print(sensor.get_euler)

if __name__ == '__main__':
    main()