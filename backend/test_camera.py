# import cv2
# import time

# # Open the default camera (usually /dev/video0)
# cap = cv2.VideoCapture('v4l2src device=/dev/video0 ! videoconvert ! appsink', cv2.CAP_GSTREAMER)


# if not cap.isOpened():
#     print("❌ Failed to open camera.")
# else:
#     ret, frame = cap.read()
#     if ret:
#         filename = f"snapshot_{int(time.time())}.jpg"
#         cv2.imwrite(filename, frame)
#         print(f"📸 Saved image as {filename}")
#     else:
#         print("⚠️ Failed to capture image.")

# cap.release()

# from picamera2 import Picamera2, Preview
# from time import sleep

# picam2 = Picamera2()

# # Set up the configuration once
# camera_config = picam2.create_preview_configuration()
# picam2.configure(camera_config)

# def live_video():
#     picam2.start_preview(Preview.QTGL)  # Show video window (requires X11)
#     picam2.start()
#     sleep(5)
#     picam2.stop_preview()
#     picam2.stop()

# def capture_image():
#     picam2.start()
#     sleep(2)  # Let camera warm up
#     picam2.capture_file("test_image.jpg")
#     picam2.stop()

# capture_image()
# picam2.close()

from picamera2 import Picamera2
import cv2

picam2 = Picamera2()
picam2.start()

frame = picam2.capture_array()  # returns a NumPy array
cv2.imwrite("snapshot.jpg", frame)
print("Saved snapshot.jpg")

picam2.stop()
picam2.close()
