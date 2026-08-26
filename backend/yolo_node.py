from ultralytics import YOLO
import cv2
import time
import sys
import threading
import atexit

from camera_mod import CameraMod


class YoloNode():
    def __init__(self):
        self.cam = CameraMod(config="yolo")
        atexit(self.cam.close) #clean shutdown
        self.frame = None
        self.lock = threading.Lock()

        # Load model
        print("[INFO] Loading YOLOv8n model...")
        self.model = YOLO("yolov8n.pt")

        # Force CPU
        self.model.to("cpu")

        threading.Thread(target=self.capture_frames, daemon=True).start()


    def run_inference(self, image):
        if image is None:
            print(f"[ERROR] Could not load image: {image}")
            sys.exit(1)

        #print("[INFO] Running inference...")
        start = time.time()

        results = self.model(
            image,
            imgsz=240,
            conf=.4,
            verbose=False
        )

        elapsed = (time.time() - start) * 1000
        #print(f"[INFO] Inference time: {elapsed:.2f} ms")

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

                # Draw box
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # Draw label
                cv2.putText(
                    annotated,
                    label,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

        return annotated


    # --- Frame capture thread ---
    def capture_frames(self):
        while True:
            img = self.cam.capture_image()
            img = self.run_inference(img)
            _, jpeg = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            with self.lock:
                self.frame = jpeg.tobytes()
            #time.sleep(0.03)  # ~30 FPS


    def mjpeg_stream(self):
        while True:
            with self.lock:
                if self.frame is None:
                    time.sleep(0.01) 
                    continue
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" +
                    self.frame +
                    b"\r\n"
                )
