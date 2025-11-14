#!/usr/bin/env python3
import os
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Transform, TransformStamped
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs2  # type: ignore
from tf2_ros import TransformBroadcaster

class DetTFNode(Node):
    """Node that listens to tag center pixels and an aligned depth image,
    deprojects the pixel to a 3D point using RealSense intrinsics, and
    publishes a TF transform in the camera frame.
    """

    def __init__(self):
        super().__init__('det_tf_node')

        # parameters
        self.declare_parameter('tag_center_topic', '/tag_center_pixel')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('intrinsics_yaml', 'src/robot_vision/config/camera_parameter.yaml')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('depth_scale', 0.0010000000474974513)

        tag_center_topic = self.get_parameter('tag_center_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        intrinsics_yaml = self.get_parameter('intrinsics_yaml').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.depth_scale = self.get_parameter('depth_scale').get_parameter_value().double_value

        # publishers/subscribers
        self.tag_center_sub = self.create_subscription(Transform, tag_center_topic, self.cal_tf_callback, 10)
        self.depth_img_sub = self.create_subscription(Image, depth_topic, self.depth_img_callback, 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.bridge = CvBridge()
        self.latest_depth = None
        self.latest_depth_header = None

        # load intrinsics
        self.rgb_camera_intrinsics = None
        if intrinsics_yaml:
            path = intrinsics_yaml
            if not os.path.isabs(path):
                candidate = os.path.join(os.getcwd(), path)
                if os.path.exists(candidate):
                    path = candidate
            try:
                with open(path, 'r') as f:
                    data = yaml.safe_load(f)
                intrinsic = data.get('camera_intrinsics', data)
                self.rgb_camera_intrinsics = rs2.intrinsics()  # type: ignore
                self.rgb_camera_intrinsics.width = int(intrinsic['width'])
                self.rgb_camera_intrinsics.height = int(intrinsic['height'])
                self.rgb_camera_intrinsics.ppx = float(intrinsic['ppx'])
                self.rgb_camera_intrinsics.ppy = float(intrinsic['ppy'])
                self.rgb_camera_intrinsics.fx = float(intrinsic['fx'])
                self.rgb_camera_intrinsics.fy = float(intrinsic['fy'])
                self.rgb_camera_intrinsics.coeffs = [float(c) for c in intrinsic.get('coeffs', [])]
                self.get_logger().info(f'Loaded intrinsics from {path}')
            except Exception as e:
                self.get_logger().warning(f'Failed to load intrinsics: {e}')

        self.get_logger().info(f'DetTFNode listening for {tag_center_topic} and depth on {depth_topic}')

    def depth_img_callback(self, msg: Image):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth = np.array(depth_image)
            self.latest_depth_header = msg.header
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')

    def cal_tf_callback(self, msg: Transform):
        if self.latest_depth is None:
            self.get_logger().warning('No depth image received yet')
            return

        px = int(round(msg.translation.x))
        py = int(round(msg.translation.y))

        h, w = self.latest_depth.shape[:2]
        if not (0 <= px < w and 0 <= py < h):
            self.get_logger().warning(f'Pixel out of bounds: {(px,py)} size {(w,h)}')
            return

        depth_val = self.latest_depth[py, px]
        depth_m = float(depth_val) * float(self.depth_scale)

        if not (depth_m > 0.0) or np.isnan(depth_m):
            self.get_logger().warning(f'Invalid depth at {(px,py)}: raw={depth_val} m={depth_m}')
            return

        if self.rgb_camera_intrinsics is None:
            self.get_logger().warning('Depth intrinsics not available; cannot deproject')
            return

        try:
            point3d = rs2.rs2_deproject_pixel_to_point(self.rgb_camera_intrinsics, [px, py], depth_m)
        except Exception as e:
            self.get_logger().error(f'Deprojection failed: {e}')
            return

        self.get_logger().info(f'Deprojected 3D point: {point3d}')

        # Create TransformStamped and broadcast TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.camera_frame
        tf_msg.child_frame_id = "tag_pose" 
        tf_msg.transform.translation.x = float(point3d[0])
        tf_msg.transform.translation.y = float(point3d[1])
        tf_msg.transform.translation.z = float(point3d[2])

        tf_msg.transform.rotation.x = msg.rotation.x
        tf_msg.transform.rotation.y = msg.rotation.y
        tf_msg.transform.rotation.z = msg.rotation.z
        tf_msg.transform.rotation.w = msg.rotation.w

        self.tf_broadcaster.sendTransform(tf_msg)
        self.get_logger().debug(f'Broadcast TF: {tf_msg}')

def main(args=None):
    rclpy.init(args=args)
    node = DetTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
