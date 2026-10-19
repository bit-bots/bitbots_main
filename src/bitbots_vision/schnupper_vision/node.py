"""ROS-Anbindung unabhängig von der austauschbaren Schülerfunktion."""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from soccer_vision_2d_msgs.msg import Ball, BallArray

from schnupper_vision.detector import detect_balls


def validate_boxes(box: tuple[int, int, int, int] | None, image_shape: tuple) -> np.ndarray:
    """Prüfe den Notebook-Vertrag und beschränke Boxen auf das Bild."""
    if box is None:
        return np.empty((0, 4), dtype=float)
    if not isinstance(box, tuple) or len(box) != 4:
        raise ValueError("detect_balls muss ein Tupel (links, oben, rechts, unten) oder None liefern")
    boxes = np.asarray([box])
    if boxes.dtype.kind not in "iuf" or not np.isfinite(boxes).all():
        raise ValueError("Boxkoordinaten müssen endliche reelle Zahlen sein")
    if np.any(boxes[:, 2:] <= boxes[:, :2]):
        raise ValueError("xyxy-Boxen benötigen positive Breite und Höhe")
    clipped = boxes.astype(np.float64, copy=True)
    height, width = image_shape[:2]
    clipped[:, [0, 2]] = np.clip(clipped[:, [0, 2]], 0, width)
    clipped[:, [1, 3]] = np.clip(clipped[:, [1, 3]], 0, height)
    return clipped[np.all(clipped[:, 2:] > clipped[:, :2], axis=1)]


def make_ball_array(boxes: np.ndarray, header) -> BallArray:
    """Übertrage Pixelboxen in denselben Nachrichtentyp wie die Original-Vision."""
    message = BallArray()
    message.header = header
    for left, top, right, bottom in boxes:
        ball = Ball()
        ball.center.x = float((left + right) / 2)
        ball.center.y = float((top + bottom) / 2)
        ball.bb.center.position.x = ball.center.x
        ball.bb.center.position.y = ball.center.y
        ball.bb.size_x = float(right - left)
        ball.bb.size_y = float(bottom - top)
        ball.confidence.confidence = 1.0
        message.balls.append(ball)
    return message


class SchnupperVision(Node):
    def __init__(self):
        super().__init__("bitbots_vision")
        image_topic = self.declare_parameter("topics.image", "zed/zed_node/rgb/image_rect_color").value
        balls_topic = self.declare_parameter("topics.balls", "balls_in_image").value
        debug_topic = self.declare_parameter("topics.debug_image", "debug_image").value
        self.declare_parameter("components.debug_active", False)
        self.bridge = CvBridge()
        self.ball_publisher = self.create_publisher(BallArray, balls_topic, 1)
        self.debug_publisher = self.create_publisher(Image, debug_topic, 1)
        image_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.image_subscription = self.create_subscription(Image, image_topic, self.image_callback, image_qos)

    def image_callback(self, image_message: Image):
        try:
            image_bgr = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        except Exception as error:
            self.get_logger().error(f"Bildkonvertierung fehlgeschlagen: {error}")
            return
        try:
            boxes = validate_boxes(detect_balls(image_bgr), image_bgr.shape)
        except Exception as error:
            self.get_logger().error(f"detect_balls fehlgeschlagen: {error}")
            boxes = np.empty((0, 4), dtype=np.float64)
        self.ball_publisher.publish(make_ball_array(boxes, image_message.header))
        if self.get_parameter("components.debug_active").value:
            overlay = image_bgr.copy()
            for left, top, right, bottom in boxes:
                cv2.rectangle(
                    overlay,
                    (int(left), int(top)),
                    (min(int(right), overlay.shape[1] - 1), min(int(bottom), overlay.shape[0] - 1)),
                    (0, 255, 0),
                    2,
                )
            debug_message = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            debug_message.header = image_message.header
            self.debug_publisher.publish(debug_message)


def main(args=None):
    rclpy.init(args=args)
    node = SchnupperVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
