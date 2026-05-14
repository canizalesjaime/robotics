#uvicorn main:app --host 0.0.0.0 --port 8000
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from pydantic import BaseModel

from robot_controller import RobotController
#from yolo_node import YoloNode
from mpu6050_node import Mpu6050Node
from big_arm import BigArmNode

app = FastAPI()
robot = RobotController()
#yolo_node = YoloNode()
accelerometer = Mpu6050Node()
arm = BigArmNode()

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

class ArmAngles(BaseModel):
    base: int
    shoulder: int
    elbow: int
    gripper: int

# @app.get("/camera")
# def camera():
#     return StreamingResponse(
#         yolo_node.mjpeg_stream(),
#         media_type="multipart/x-mixed-replace; boundary=frame"
#     )

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

@app.post("/set_angles")
def set_angles(angs: ArmAngles):
    arm.set_angles_api([angs.base,angs.shoulder,angs.elbow,angs.gripper])

@app.on_event("shutdown")
def shutdown():
    robot.shutdown()
