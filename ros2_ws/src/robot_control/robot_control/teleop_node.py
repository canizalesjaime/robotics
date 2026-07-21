import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import curses
import time

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.cmd_pub = self.create_publisher(String, 'cmd_motor', 10)
        self.distance = 100.0
        self.create_subscription(Float32, 'distance', self.distance_callback, 10)

        self.current_command = None
        self.last_keypress_time = 0

    def distance_callback(self, msg):
        self.distance = msg.data

    def control_loop(self, stdscr):
        stdscr.nodelay(True)
        stdscr.addstr(0, 0, "Hold 'a' to move forward, 'z' backward, 'l' rotate left, 'r' rotate right, 'x' to stop. 'u' to increase speed and 'd' to decrease speed")

        rate_hz = 10
        delay = 1.0 / rate_hz
        key_hold_timeout = 0.5  # seconds


        while rclpy.ok():
            try:
                key = stdscr.getch()
                now = time.time()

                if key == ord('a') and self.distance > 8.0:
                    self.current_command = 'forward'
                    self.last_keypress_time = now
                elif key == ord('z'):
                    self.current_command = 'backward'
                    self.last_keypress_time = now
                elif key == ord('l') and self.distance > 8.0:
                    self.current_command = 'rotate_left'
                    self.last_keypress_time = now
                elif key == ord('r') and self.distance > 8.0:
                    self.current_command = 'rotate_right'
                    self.last_keypress_time = now
                elif key == ord('x'):
                    self.current_command = 'stop'
                    self.last_keypress_time = now
                elif key == ord('u'):
                    self.current_command = 'increase'
                    self.last_keypress_time = now
                elif key == ord('d'):
                    self.current_command = 'decrease'
                    self.last_keypress_time = now
                elif key != -1:
                    # Unknown key — treat it as stop for safety
                    self.current_command = 'stop'
                    self.last_keypress_time = now

                # If no key pressed for too long, stop
                if now - self.last_keypress_time > key_hold_timeout:
                    if self.current_command != 'stop':
                        self.current_command = 'stop'

                # Publish command
                if self.current_command:
                    self.cmd_pub.publish(String(data=self.current_command))

                #time.sleep(delay)
                rclpy.spin_once(self, timeout_sec=delay)

            except KeyboardInterrupt:
                break

def main():
    rclpy.init()
    node = TeleopNode()
    try:
        curses.wrapper(node.control_loop)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()