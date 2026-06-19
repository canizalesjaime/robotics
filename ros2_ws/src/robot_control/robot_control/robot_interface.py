from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
#from contextlib import asynccontextmanager
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32


app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # tighten later
    allow_methods=["*"],
    allow_headers=["*"],
)

# bridge = None
# stop_event = threading.Event()

class Command(BaseModel):
    action: str

class Speed(BaseModel):
    speed: int

class ArmAngles(BaseModel):
    base: int
    shoulder: int
    elbow: int
    gripper: int

class RobotInterface(Node):
    def __init__(self):
        super().__init__('robot_interface_node')
        #tt motor
        self.cmd_pub = self.create_publisher(String, 'cmd_motor', 10)

        # lidar
        self.create_subscription(Float32, 'distance', self.distance_callback, 10)

        self.distance = None
        self.current_command = None
        

    def distance_callback(self, msg):
        self.distance=msg.data
        print(msg.data)


@app.post("/command")
def send_command(cmd: Command):
    robot.command(cmd.action)
    return {"ok": True}

@app.post("/speed")
def set_speed(spd: Speed):
    robot.set_speed(spd.speed)
    return {"ok": True, "speed": spd.speed}

@app.get("/status")
def get_status():
    return robot.status()

@app.get("/accelerometer")
def get_accelerometer_data():
    return accelerometer.sensor_data()

@app.get("/lidar")
def get_lidar_data():
    distance, strength = lidar.get_distance()
    return {
        "distance": distance,
        "strength": strength
    }

@app.post("/set_angles")
def set_angles(angs: ArmAngles):
    arm.set_angles_api([angs.base,angs.shoulder,angs.elbow,angs.gripper])

@app.get("/rotate_base")
def rotate_base():
    arm.rotate_base()
    return {"status": "rotating"}

@app.get("/stop_base")
def stop_base():
    arm.stop_base()
    return {"status": "stopped"}


@app.on_event("shutdown")
def shutdown():
    robot.shutdown()


def ros_spin():
    rclpy.spin(bridge)


rclpy.init()

bridge = RobotInterface()

threading.Thread(
    target=ros_spin,
    daemon=True
).start()




# below considers ros2 cleanup, if I want to go hard about that

# def ros_spin():
#     while rclpy.ok() and not stop_event.is_set():
#         rclpy.spin_once(bridge, timeout_sec=0.1)


# # --------------------
# # FastAPI lifecycle (startup/shutdown)
# # --------------------

# @asynccontextmanager
# async def lifespan(app: FastAPI):
#     global bridge

#     # ---- startup ----
#     rclpy.init()
#     bridge = RobotInterface()

#     thread = threading.Thread(
#         target=ros_spin,
#         daemon=False  # important: NOT daemon for clean shutdown
#     )
#     thread.start()

#     yield  # app runs here

#     # ---- shutdown ----
#     stop_event.set()

#     bridge.destroy_node()
#     rclpy.shutdown()

#     thread.join(timeout=2.0)


# app = FastAPI(lifespan=lifespan)