import serial
import time


class LunarNode():
    def __init__(self):
        self.ser = serial.Serial("/dev/ttyAMA0", 115200, timeout=1)
        self.distance=-1
        self.strength=-1
        print("Reading TF-Luna data...")

    def get_distance(self):
        if self.ser.in_waiting >= 9:
            data = self.ser.read(9)
            # TF-Luna frame starts with 0x59 0x59
            if data[0] == 0x59 and data[1] == 0x59:
               self.distance = data[2] + data[3] * 256
               self.strength = data[4] + data[5] * 256
        return (self.distance/100, self.strength) # in inches


def main():
    node = LunarNode()
    try:
        while True:
            d,s=node.get_distance()
            print(f"Distance: {d} cm | Strength: {s}")

    except KeyboardInterrupt:
        print("\nExiting...")
    
    # finally:
    #     node.free_gpio()

if __name__ == '__main__':
    main()
