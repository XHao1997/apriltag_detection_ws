#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CameraInfo

from cv_bridge import CvBridge
import numpy as np
import pyrealsense2 as rs


class TreeBaseTo3DNode(Node):
    def __init__(self):
        super().__init__('tree_base_to_3d_node')

        self.get_logger().info('TreeBaseTo3DNode (pyrealsense deproject) started.')

        self.bridge = CvBridge()

        # Latest depth image (aligned to color) and its header
        self.depth_image = None          # numpy array
        self.depth_header = None

        # RealSense intrinsics for the color camera
        self.intrinsics = None           # rs.intrinsics

        # Depth scale: meters per depth unit for uint16 depth
        # e.g., RealSense default: 0.001 for mm; adjust if needed or make a ROS param
        self.depth_scale = 9.999999747378752e-05

        # Subscriptions
        # 1) Depth image aligned to color
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        # 2) Color camera intrinsics
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        # 3) 2D tree base position in RGB image (pixel coordinates)
        self.tree_sub = self.create_subscription(
            PointStamped,
            '/tree_base_pose',
            self.tree_callback,
            10
        )

        # Publisher: 3D point for RViz2
        self.point_3d_pub = self.create_publisher(
            PointStamped,
            '/tree_base_point_3d',
            10
        )

    # ----------------- Callbacks -----------------

    def depth_callback(self, msg: Image):
        """Store latest aligned depth image as numpy array."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')
            return

        self.depth_image = np.array(cv_image)
        self.depth_header = msg.header

    def camera_info_callback(self, msg: CameraInfo):
        """Convert CameraInfo to rs.intrinsics (once)."""
        if self.intrinsics is not None:
            # Intrinsics usually don't change at runtime
            return

        intr = rs.intrinsics()
        intr.width = msg.width
        intr.height = msg.height
        intr.ppx = msg.k[2]  # cx
        intr.ppy = msg.k[5]  # cy
        intr.fx = msg.k[0]   # fx
        intr.fy = msg.k[4]   # fy

        if msg.distortion_model == "plumb_bob":
            intr.model = rs.distortion.brown_conrady
        elif msg.distortion_model == "equidistant":
            intr.model = rs.distortion.kannala_brandt4
        else:
            intr.model = rs.distortion.none

        coeffs = list(msg.d)
        if len(coeffs) < 5:
            coeffs += [0.0] * (5 - len(coeffs))
        intr.coeffs = coeffs[:5]

        self.intrinsics = intr
        self.get_logger().info(
            f'Camera intrinsics set: fx={intr.fx:.2f}, fy={intr.fy:.2f}, '
            f'cx={intr.ppx:.2f}, cy={intr.ppy:.2f}'
        )

    def tree_callback(self, tree_msg: PointStamped):
        """Convert 2D pixel (u,v) in color image to 3D point using depth + intrinsics."""
        if self.depth_image is None:
            self.get_logger().warn('No depth image received yet, cannot compute 3D point.')
            return

        if self.intrinsics is None:
            self.get_logger().warn('No camera intrinsics yet, cannot compute 3D point.')
            return

        print("tree_callback start")

        # Interpret tree_base_pose.point.x, .y as pixel coordinates (u, v)
        u = int(tree_msg.point.x)
        v = int(tree_msg.point.y)

        H, W = self.depth_image.shape[:2]
        print(f"pixel (u,v)=({u},{v}), depth image size=({W}x{H})")

        # Boundary check
        if not (0 <= u < W and 0 <= v < H):
            self.get_logger().warn(
                f'tree_base_pose pixel out of range: u={u}, v={v}, width={W}, height={H}'
            )
            return

        # Get raw depth value
        depth_raw = self.depth_image[v, u]

        # Handle invalid depth (0 or NaN)
        if depth_raw == 0 or (isinstance(depth_raw, float) and math.isnan(depth_raw)):
            self.get_logger().warn(
                f'Depth at (u,v)=({u},{v}) is invalid (0 or NaN), cannot deproject.'
            )
            return

        # Convert to meters
        if np.issubdtype(self.depth_image.dtype, np.integer):
            depth_m = float(depth_raw) * self.depth_scale
        else:
            # If already in meters (float32)
            depth_m = float(depth_raw)

        # Deproject pixel to 3D (X,Y,Z) in camera color frame, meters
        point_3d = rs.rs2_deproject_pixel_to_point(
            self.intrinsics,
            [float(u), float(v)],
            depth_m
        )
        x, y, z = map(float, point_3d)

        if any(math.isnan(val) for val in (x, y, z)):
            self.get_logger().warn(
                f'Deprojected point at (u,v)=({u},{v}) is NaN, cannot publish 3D point.'
            )
            return

        # Build PointStamped in the aligned depth / color frame
        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = (
            self.depth_header.frame_id if self.depth_header
            else "camera_color_optical_frame"
        )
        out.point.x = x
        out.point.y = y
        out.point.z = z

        self.point_3d_pub.publish(out)

        self.get_logger().info(
            f'Published 3D tree base point: '
            f'pixel=({u},{v}) -> XYZ=({x:.3f}, {y:.3f}, {z:.3f}) in {out.header.frame_id}'
        )
        print("tree_callback end")


def main(args=None):
    rclpy.init(args=args)
    node = TreeBaseTo3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
