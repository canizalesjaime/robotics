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
bridge = None
# stop_event = threading.Event()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # tighten later
    allow_methods=["*"],
    allow_headers=["*"],
)

class Command(BaseModel):
    action: str

class Speed(BaseModel):
    speed: int

# class ArmAngles(BaseModel):
#     base: int
#     shoulder: int
#     elbow: int
#     gripper: int

class RobotInterface(Node):
    def __init__(self):
        super().__init__('robot_interface_node')
        #tt motor
        self.cmd_pub = self.create_publisher(String, 'cmd_motor', 10)
        self.speed_pub = self.create_publisher(Int32, 'cmd_speed', 10)
        
        # bno055 accelerometer:imu 
        self.create_subscription(Imu,'imu/data_raw',self.imu_callback,10)
        
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8000)

        self.host = self.get_parameter("host").value
        self.port = self.get_parameter("port").value

        self.robot_state = {
            "imu": {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            },
            "motor_speed": 30,
            #"battery": 0.0,
        }

    def imu_callback(self, msg):
        roll, pitch, yaw = euler_from_quaternion([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ])

        self.robot_state["imu"]["roll"] = roll
        self.robot_state["imu"]["pitch"] = pitch
        self.robot_state["imu"]["yaw"] = yaw


@app.get("/robot_state")
def get_robot_state():
    return bridge.robot_state


@app.post("/command")
def send_command(cmd: Command):
    cmd_map = {"forward": "f", "backward":"b", "left":"l", "right":"r", 
               "rotate_left":"rl", "rotate_right":"rr", "stop":"s"}

    if cmd.action in cmd_map:
        cmd_msg=String()
        cmd_msg.data=cmd_map[cmd.action]
        bridge.cmd_pub.publish(cmd_msg)
    return {"ok": True}


@app.post("/speed")
def set_speed(spd: Speed):
    spd_msg= Int32()
    spd_msg.data=spd.speed
    bridge.speed_pub.publish(spd_msg)
    return {"ok": True, "speed": spd.speed}

# @app.post("/set_angles")
# def set_angles(angs: ArmAngles):
#     arm_msg = JointState()

#     arm_msg.name = ["base","shoulder","elbow","gripper"]

#     arm_msg.position = [math.radians(angs.base),math.radians(angs.shoulder),
#                         math.radians(angs.elbow),math.radians(angs.gripper)]

#     bridge.arm_pub.publish(arm_msg)
    

@app.post("/rotate_base")
def rotate_base():
    bridge.robot_state["base_rotating"] = True
    b_msg=Bool()
    b_msg.data=True
    bridge.base_pub.publish(b_msg)
    return {"status": "rotating"}


@app.post("/stop_base")
def stop_base():
    bridge.robot_state["base_rotating"] = False
    b_msg=Bool()
    b_msg.data=False
    bridge.base_pub.publish(b_msg)
    return {"status": "not rotating"}


def ros_spin():
    while rclpy.ok():
        rclpy.spin_once(bridge,timeout_sec=0.1)


def main():
    global bridge
    rclpy.init()
    bridge = RobotInterface()
    threading.Thread(
        target=ros_spin,
        daemon=True
    ).start()

    import uvicorn

    uvicorn.run(
        app,
        host=bridge.host,
        port=bridge.port
    )


if __name__ == "__main__":
    main()

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
