#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32

from cv_bridge import CvBridge
import cv2

from ultralytics.models import YOLO  # 如果你用的是 from ultralytics import YOLO 也可以
import torch
import os


class TreeBasePoseNode(Node):
    def __init__(self):
        super().__init__('tree_base_pose_node')
        print(os.getcwd())
        # Parameters
        self.declare_parameter('model_path', 'src/ros2_rm_robot/apriltag_detection/weights/best.pt')
        self.declare_parameter('image_topic', '/camera/camera/color/image_rect_raw')
        self.declare_parameter('keypoint_index', 0)  # which keypoint in the skeleton
        self.declare_parameter('debug', False)       # 是否画点并发布 debug 图像

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.keypoint_index = self.get_parameter('keypoint_index').get_parameter_value().integer_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value

        self.get_logger().info(f'Loading YOLO pose model from: {model_path}')
        self.model = YOLO(model_path)

        # Optional: pick device automatically (GPU if available)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Using device: {self.device}')
        self.model.to(self.device)

        self.bridge = CvBridge()

        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        # Publish pixel point
        self.pose_pub = self.create_publisher(
            PointStamped,
            '/tree_base_pose',
            10
        )

        # Publish detection flag: 1 = detected, 0 = not detected
        self.detect_pub = self.create_publisher(
            Int32,
            '/tree_base_detection',
            10
        )

        # debug image
        if self.debug:
            self.debug_image_pub = self.create_publisher(
                Image,
                '/tree_keypoint',
                10
            )
            self.get_logger().info('Debug mode ON: publishing annotated image to /tree_keypoint')
        else:
            self.debug_image_pub = None
            self.get_logger().info('Debug mode OFF')

        self.get_logger().info(
            f'Subscribing to image topic: {image_topic}, publishing keypoint {self.keypoint_index} to /tree_base_pose'
        )

    # helper: publish detection flag
    def publish_detection_flag(self, value: int):
        msg = Int32()
        msg.data = value
        self.detect_pub.publish(msg)

    # helper: when no detection, publish image with "None" text
    def publish_debug_none(self, cv_image, header, reason: str):
        if not self.debug or self.debug_image_pub is None:
            return
        img = cv_image.copy()
        text = f'None ({reason})'
        cv2.putText(
            img,
            text,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 255),
            2
        )
        debug_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        debug_msg.header = header
        self.debug_image_pub.publish(debug_msg)

    def image_callback(self, msg: Image):
        # Convert ROS Image to OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # Run YOLO pose inference
        results = self.model(cv_image, conf=0.15, verbose=False)

        if len(results) == 0:
            self.get_logger().warn('YOLO returned no results')
            self.publish_detection_flag(0)
            self.publish_debug_none(cv_image, msg.header, 'no results')
            return

        result = results[0]

        # result.keypoints.xy is [num_dets, num_kpts, 2] in pixel coordinates (x, y)
        if result.keypoints is None or result.keypoints.xy is None:
            self.get_logger().warn('No keypoints found in detection')
            self.publish_detection_flag(0)
            self.publish_debug_none(cv_image, msg.header, 'no keypoints')
            return

        kpts_xy = result.keypoints.xy  # tensor
        num_dets, num_kpts, _ = kpts_xy.shape

        if num_dets == 0:
            self.get_logger().warn('No detections found')
            self.publish_detection_flag(0)
            self.publish_debug_none(cv_image, msg.header, 'num_dets=0')
            return

        if self.keypoint_index < 0 or self.keypoint_index >= num_kpts:
            self.get_logger().error(
                f'keypoint_index {self.keypoint_index} out of range (num_kpts={num_kpts})'
            )
            self.publish_detection_flag(0)
            self.publish_debug_none(cv_image, msg.header, 'index out of range')
            return

        # Take first detection for now
        kp = kpts_xy[0, self.keypoint_index]  # shape [2] -> (x, y) in pixels
        u = float(kp[0].item())
        v = float(kp[1].item())

        # We have a valid detection
        self.publish_detection_flag(1)

        # Build and publish PointStamped
        point_msg = PointStamped()
        point_msg.header = msg.header  # same timestamp / frame_id as image
        point_msg.point.x = u
        point_msg.point.y = v
        point_msg.point.z = 0.0

        self.pose_pub.publish(point_msg)
        self.get_logger().debug(f'Published keypoint {self.keypoint_index} at (u={u:.1f}, v={v:.1f})')

        # ---------- debug: 在图像上画出 keypoint 并发布 ----------
        if self.debug and self.debug_image_pub is not None:
            # 画圆点
            cv2.circle(
                cv_image,
                (int(u), int(v)),
                6,
                (0, 0, 255),  # 红色
                -1
            )
            # 写一点文字
            cv2.putText(
                cv_image,
                f'kp{self.keypoint_index} ({int(u)}, {int(v)})',
                (int(u) + 5, int(v) - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1
            )

            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            debug_msg.header = msg.header  # 保持时间戳和 frame_id 一致
            self.debug_image_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TreeBasePoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
