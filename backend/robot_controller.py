# Teleop node
import time
import threading
from motor_node import MotorNode
from ultrasonic import UltrasonicNode

class RobotController():
    def __init__(self):
        self.current_action = None
        self.dist = 0
        self.lock = threading.Lock()
        self.stop_event = threading.Event()

        self.motor = MotorNode()
        self.ultra = UltrasonicNode()

        self.monitor_thread = threading.Thread(
            target=self.distance_monitor,
            daemon=True
        )
        self.monitor_thread.start()

    def distance_monitor(self):
        while not self.stop_event.is_set():
            self.dist = self.ultra.get_distance()
            with self.lock:
                if self.current_action == "forward" and self.dist <= 11.0:
                    print("Obstacle detected! Stopping.")
                    self.motor.move("s")
                    self.current_action = None
            time.sleep(0.2)
    
    def set_speed(self, speed):
        print(f"Setting speed to {speed}")
        with self.lock:
            self.motor.set_speed(speed)

    def command(self, action):
        with self.lock:
            if action == "forward" and self.dist > 8:
                self.motor.move("f")
                self.current_action = "forward"

            elif action == "backward":
                self.motor.move("b")
                self.current_action = "backward"

            elif action == "left":
                self.motor.move("l")
                self.current_action = "left"

            elif action == "right":
                self.motor.move("r")
                self.current_action = "right"

            elif action == "rotate_left":
                self.motor.move("rl")
                self.current_action = "rotate_left"

            elif action == "rotate_right":
                self.motor.move("rr")
                self.current_action = "rotate_right"

            elif action == "stop":
                self.motor.move("s")
                self.current_action = None

    def status(self):
        return {
            "action": self.current_action,
            "distance": self.dist
        }

    def shutdown(self):
        self.stop_event.set()
        self.monitor_thread.join()
        self.motor.release_lines()
        self.ultra.free_gpio()
