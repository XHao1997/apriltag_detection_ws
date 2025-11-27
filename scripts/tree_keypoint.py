#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge
import cv2

from ultralytics import YOLO
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

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.keypoint_index = self.get_parameter('keypoint_index').get_parameter_value().integer_value

        self.get_logger().info(f'Loading YOLO pose model from: {model_path}')
        self.model = YOLO(model_path)

        # Optional: pick device automatically (GPU if available)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Using device: {self.device}')
        self.model.to(self.device)

        self.bridge = CvBridge()

        # Subscriber and publisher
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.pose_pub = self.create_publisher(
            PointStamped,
            '/tree_base_pose',
            10
        )

        self.get_logger().info(
            f'Subscribing to image topic: {image_topic}, publishing keypoint {self.keypoint_index} to /tree_base_pose'
        )

    def image_callback(self, msg: Image):
        # Convert ROS Image to OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # Run YOLO pose inference
        # NOTE: ultralytics expects BGR np.array; this is fine
        results = self.model(cv_image, verbose=False)

        if len(results) == 0:
            self.get_logger().warn('YOLO returned no results')
            return

        result = results[0]

        # result.keypoints.xy is [num_dets, num_kpts, 2] in pixel coordinates (x, y)
        if result.keypoints is None or result.keypoints.xy is None:
            self.get_logger().warn('No keypoints found in detection')
            return

        kpts_xy = result.keypoints.xy  # tensor
        num_dets, num_kpts, _ = kpts_xy.shape

        if num_dets == 0:
            self.get_logger().warn('No detections found')
            return

        if self.keypoint_index < 0 or self.keypoint_index >= num_kpts:
            self.get_logger().error(
                f'keypoint_index {self.keypoint_index} out of range (num_kpts={num_kpts})'
            )
            return

        # Take first detection for now
        kp = kpts_xy[0, self.keypoint_index]  # shape [2] -> (x, y) in pixels
        u = float(kp[0].item())
        v = float(kp[1].item())

        # Build and publish PointStamped
        point_msg = PointStamped()
        point_msg.header = msg.header  # same timestamp / frame_id as image
        point_msg.point.x = u
        point_msg.point.y = v
        point_msg.point.z = 0.0

        self.pose_pub.publish(point_msg)
        self.get_logger().debug(f'Published keypoint {self.keypoint_index} at (u={u:.1f}, v={v:.1f})')

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
