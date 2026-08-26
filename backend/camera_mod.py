from picamera2 import Picamera2, Preview
import cv2
import time


class CameraMod():
    def __init__(self,config=""):
        # Open the default camera (usually /dev/video0)
        # cap = cv2.VideoCapture('v4l2src device=/dev/video0 ! videoconvert ! appsink', cv2.CAP_GSTREAMER)   
        self.picam2 = Picamera2()
        self.set_config(config)


    def set_config(self, config):
        if config=="yolo":
            config = self.picam2.create_video_configuration(
                        main={"size": (320, 240), "format": "RGB888"} )
        else:
            config = self.picam2.create_preview_configuration()
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(2)  # Let camera warm up


    def live_video(self):
        self.picam2.start_preview(Preview.QTGL)  # Show video window (requires X11)
        
    

    def capture_image(self):     
        #self.picam2.capture_file("test_image.jpg")
        frame = self.picam2.capture_array()  # returns a NumPy array
        #cv2.imwrite("test_image.jpg", frame)
        return frame

    # def open_cv_camera_alternative(self):
    #     if not self.cap.isOpened():
    #         print("Failed to open camera.")
    #     else:
    #         ret, frame = self.cap.read()
    #         if ret:
    #             filename = f"snapshot_{int(time.time())}.jpg"
    #             cv2.imwrite(filename, frame)
    #             print(f"Saved image as {filename}")
    #         else:
    #             print("Failed to capture image.")

    #     self.cap.release()

    def close(self):
        self.picam2.stop()
        self.picam2.stop_preview()
        self.picam2.close()


def main():
    cam = CameraMod()
    cam.capture_image()
    # I want to add this to be called when object is destroyed
    cam.close()


if __name__=='__main__':
    main()
