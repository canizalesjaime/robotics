from picamera2 import Picamera2, Preview
from time import sleep

picam2 = Picamera2()

def live_video():
    picam2.start_preview(Preview.QTGL)
    picam2.start()
    sleep(5)


def capture_image():
    # camera_config = picam2.create_preview_configuration()
    # picam2.configure(camera_config)
    # picam2.start_preview(Preview.NULL)
    picam2.start()
    sleep(2)
    picam2.capture_file("test1.jpg")



capture_image()
picam2.close()