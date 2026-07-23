# should I separate, ros2 and fastapi?
# post:publish, get:subscribe
# you must run this using uvicorn and not ros: 
# uvicorn robot_interface:app --host 0.0.0.0 --port 8000
import math

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
#from contextlib import asynccontextmanager
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool,Int32
from sensor_msgs.msg import Imu, Temperature, JointState
from tf_transformations import euler_from_quaternion


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
        self.speed_pub = self.create_publisher(Int32, 'cmd_speed', 10)
        # servo motors for arm
        self.arm_pub = self.create_publisher(JointState, 'cmd_arm', 10)
        self.base_pub = self.create_publisher(Bool, 'cmd_base', 10)

        # lidar
        self.create_subscription(Float32,'li_distance',self.distance_callback,10)
        # accelerometer: mpu6050
        self.create_subscription(Imu,'imu/data_raw',self.imu_callback,10)

        self.distance = None
        self.imu = None
        

    def distance_callback(self, msg):
        self.distance=msg.data

    def imu_callback(self, msg):
        self.imu = msg
    

@app.get("/accelerometer")
def get_accelerometer_data():
    roll, pitch, yaw = euler_from_quaternion([bridge.imu.x,bridge.imu.y,bridge.imu.z,bridge.imu.w])
    return {"x":roll,"y":pitch,"z":0} # roll(x), pitch(y),temp(z)

@app.get("/lidar")
def get_lidar_data():
    distance = bridge.distance
    return {"distance": distance,"strength": -1}

@app.post("/command")
def send_command(cmd: Command):
    cmd_map = {"forward": "f", "backward":"b", "left":"l", "right":"r", 
               "rotate_left":"rl", "rotate_right":"rr", "stop":"s"}

    if cmd.action in cmd_map:
        cmd_msg=String()
        cmd_msg.data=cmd[cmd.action]
        bridge.cmd_pub.publish(cmd_msg)
    return {"ok": True}

@app.post("/speed")
def set_speed(spd: Speed):
    spd_msg= Int32()
    spd_msg.data=spd.speed
    bridge.speed_pub.publish(spd_msg)
    return {"ok": True, "speed": spd.speed}

@app.post("/set_angles")
def set_angles(angs: ArmAngles):
    arm_msg = JointState()

    arm_msg.name = ["base","shoulder","elbow","gripper"]

    arm_msg.position = [math.radians(angs.base),math.radians(angs.shoulder),
                        math.radians(angs.elbow),math.radians(angs.gripper)]

    bridge.arm_pub.publish(arm_msg)
    

@app.post("/rotate_base")
def rotate_base():
    b_msg=Bool()
    b_msg.data=True
    bridge.base_pub.publish(b_msg)
    return {"status": "rotating"}

@app.get("/stop_base")
def stop_base():
    b_msg=Bool()
    b_msg.data=False
    bridge.base_pub.publish(b_msg)
    return {"status": "not rotating"}


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