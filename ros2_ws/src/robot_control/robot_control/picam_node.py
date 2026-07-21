import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from picamera2 import Picamera2
import cv2
import time

class PicamNode(Node):
    def __init__(self):
        super().__init__('picam_node')

        # Picamera setup
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)})
        self.picam2.configure(config)
        self.picam2.start()

        # ROS setup
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.publish_frame)  # 10 Hz
        self.srv = self.create_service(Trigger, 'capture_image', self.capture_image_callback)

        self.get_logger().info("Picamera2 node started.")

    def publish_frame(self):
        frame = self.picam2.capture_array()
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
        self.publisher_.publish(msg)

    def capture_image_callback(self, request, response):
        filename = f"/tmp/snapshot_{int(time.time())}.jpg"
        self.picam2.capture_file(filename)
        response.success = True
        response.message = f"Image saved to {filename}"
        self.get_logger().info(response.message)
        return response

    def destroy_node(self):
        self.picam2.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PicamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()