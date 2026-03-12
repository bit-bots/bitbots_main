import rclpy
from rclpy.node import Node
import cv2
import yaml
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rclpy.duration import Duration

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_driver_node')
        
        D = [-0.36272164090180586, 0.13015384929430587, 0.003100213654583986, -0.001484505800315495, 0.0]
        K = [481.8601409826828, 0.0, 325.8762965500646, 0.0, 482.5058596863874, 205.2206496766716, 0.0, 0.0, 1.0]
        R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        P = [393.74243416462537, 0.0, 326.6481383705857, 0.0, 0.0, 432.7540124626434, 199.024655863416, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.cam_info = CameraInfo()
        self.cam_info.width = 640
        self.cam_info.height = 480
        self.cam_info.distortion_model = 'plumb_bob'
        self.cam_info.d = D
        self.cam_info.k = K
        self.cam_info.r = R
        self.cam_info.p = P
        self.cam_info.header.frame_id = 'camera_optical_frame'
        # Parameters
        self.device_id = 0
        self.width = 640
        self.height = 480
        self.frame_id = 'camera_optical_frame'
        
        # Initialize OpenCV VideoCapture
        self.cap = cv2.VideoCapture("/dev/video6")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        # Publishers
        self.image_pub = self.create_publisher(Image, 'camera/image_proc', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        
        # Bridge and Info Manager
        self.bridge = CvBridge()

        # Timer for the loop (10 FPS)
        self.timer = self.create_timer(1.0 / 10.0, self.capture_and_publish)
        self.get_logger().info("Camera node started, capturing at 640x480 YUYV")

    def capture_and_publish(self):
        ret, frame = self.cap.read()
        t = self.get_clock().now()
        delay = Duration(seconds=0.1)  # Adjust this value based on the expected capture time
        stamp = (t - delay).to_msg()  # Add the delay to the current


        if not ret:
            self.get_logger().warn("Failed to grab frame")
            return

        # 1. Convert YUYV (implicit in OpenCV read) to RGB
        # Note: OpenCV usually reads as BGR. If raw YUYV is forced, 
        # use cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUYV)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # 2. Rotate 180 degrees
        rotated_frame = cv2.rotate(rgb_frame, cv2.ROTATE_180)

        # 4. Prepare Image Message
        img_msg = self.bridge.cv2_to_imgmsg(rotated_frame, encoding="rgb8")
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.frame_id

        # 5. Prepare Camera Info Message
        self.cam_info.header.stamp = stamp

        # Publish
        self.image_pub.publish(img_msg)
        self.info_pub.publish(self.cam_info)

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
