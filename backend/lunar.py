import serial


class LunarNode():
    def __init__(self):
        self.ser = serial.Serial("/dev/ttyAMA0", 115200, timeout=1)
        self.distance=-1
        self.strength=-1
        print("Reading TF-Luna data...")

    def get_distance(self):
        #print(self.ser.in_waiting)
        if self.ser.in_waiting >= 9:
            data = self.ser.read(9)
            # TF-Luna frame starts with 0x59 0x59
            if data[0] == 0x59 and data[1] == 0x59:
               self.distance = data[2] + data[3] * 256
               self.strength = data[4] + data[5] * 256
        return (self.distance, self.strength) # in cm


def main():
    node = LunarNode()
    try:
        for i in range(1000):
            d,s=node.get_distance()
            print(f"Distance: {d} cm | Strength: {s}")

    except KeyboardInterrupt:
        print("\nExiting...")
    
    # finally:
    #     node.free_gpio()

if __name__ == '__main__':
    main()