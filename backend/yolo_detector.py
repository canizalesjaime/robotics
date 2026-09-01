from ultralytics import YOLO
import cv2
import time
import sys
import atexit

from camera_mod import CameraMod


class YoloNode():
    def __init__(self):
        self.cam = CameraMod(config="yolo")
        atexit(self.cam.close)
        self.model = YOLO("yolov8n.pt")
        self.model.to("cpu")


    def run_inference(self, image):
        if image is None:
            print(f"[ERROR] Could not load image: {image}")
            sys.exit(1)

        start = time.time()
        results = self.model(image,imgsz=240,conf=.4,verbose=False)
        elapsed = (time.time() - start) * 1000
        annotated = image.copy()

        for r in results:
            boxes = r.boxes
            if boxes is None:
                continue

            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                label = f"{self.model.names[cls]} {conf:.2f}"

                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

                cv2.putText(annotated,label,(x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), 2)

        return annotated

    def get_image(self):
        return self.cam.capture_image()
