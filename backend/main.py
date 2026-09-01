#uvicorn main:app --host 0.0.0.0 --port 8000
import cv2
import threading
import time
import atexit
from fastapi.responses import StreamingResponse

from imu_bn0055_sensor import ImuSensor
from robot_controller import RobotController
from arm import ArmNode
from fast_models import app, Command, Speed, ArmAngles
from yolo_detector import YoloDetector

#inputs
accelerometer = ImuSensor()
yolo_detector = YoloDetector()

#outputs
robot = RobotController()
arm = ArmNode()


frame = None
lock = threading.Lock()


def capture_frames():
    global frame
    while True:
        img = yolo_detector.run_inference(yolo_detector.get_image())
        _, jpeg = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        with lock:
            frame = jpeg.tobytes()
        time.sleep(0.03)  # ~30 FPS

threading.Thread(target=capture_frames, daemon=True).start()

# --- MJPEG stream ---
def mjpeg_stream():
    while True:
        with lock:
            if frame is None:
                continue
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" +
                frame +
                b"\r\n"
            )


@app.get("/camera")
def camera():
    return StreamingResponse(
        mjpeg_stream(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

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