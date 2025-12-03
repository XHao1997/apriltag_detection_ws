#!/usr/bin/env python3
import math

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker
import os
import sys

# Ensure the project root (parent of the apriltag_detection package) is on sys.path
_current_dir = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.abspath(os.path.join(_current_dir, '..', '..'))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

from apriltag_detection.scripts.utils.cloud_segmentation import segment_2dbox



class CenterPointFromCloud(Node):
    def __init__(self):
        super().__init__('center_point_from_cloud')

        # Subscribe to your aligned point cloud
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/camera/pointcloud2',   # adjust if different
            self.pointcloud_callback,
            10,
        )

        # Publishes the 3D center point
        self.center_point_pub = self.create_publisher(
            PointStamped,
            '/center_point',
            10,
        )

        # Publishes a Marker so you can see the center point in RViz
        self.marker_pub = self.create_publisher(
            Marker,
            '/center_point_marker',
            10,
        )

        # Publishes the segmented window as a PointCloud2
        self.segment_cloud_pub = self.create_publisher(
            PointCloud2,
            '/segment_pointcloud',
            10,
        )

        # Sliding window config (in pixels)
        self.window_h = 10
        self.window_w = 20
        # stride = 1 pixel → truly sliding one by one
        self.stride_h = 20
        self.stride_w = 20

        # Current window top-left (we’ll update per callback)
        self.cur_u0 = 0
        self.cur_v0 = 0

        self.get_logger().info('CenterPointFromCloud (with sliding window) node started.')

    def pointcloud_callback(self, msg: PointCloud2):
        # Check organized cloud
        if msg.height == 1:
            self.get_logger().warn(
                'PointCloud2 is unorganized (height=1). '
                'Sliding window in (u, v) may not make sense.'
            )

        width = msg.width
        height = msg.height

        if width == 0 or height == 0:
            return

        # --- Read entire cloud as numpy (N,3) then reshape to (H, W, 3) ---
        points = pc2.read_points_numpy(
            msg,
            field_names=(['x', 'y', 'z']),
            skip_nans=False,
        )

        if points.size == 0:
            return

        try:
            points_2d = points.reshape((height, width, 3))
        except ValueError:
            self.get_logger().warn(
                f'Cannot reshape points to (H,W,3). Got shape {points.shape}, '
                f'height={height}, width={width}.'
            )
            return

        # --- Update sliding window (top-left) ---
        u0 = self.cur_u0
        v0 = self.cur_v0

        # Compute center of this window in pixel coordinates
        u_center = u0 + self.window_w // 2
        v_center = v0 + self.window_h // 2

        # Ensure window stays inside the image bounds
        if u0 + self.window_w > width:
            u0 = max(0, width - self.window_w)
        if v0 + self.window_h > height:
            v0 = max(0, height - self.window_h)
        u_center = u0 + self.window_w // 2
        v_center = v0 + self.window_h // 2

        # For safety, clamp center to image
        if not (0 <= u_center < width and 0 <= v_center < height):
            self.get_logger().warn(
                f'Center (u,v)=({u_center},{v_center}) out of bounds, width={width}, height={height}'
            )
            return

        # --- Get center 3D point ---
        cx, cy, cz = map(float, points_2d[v_center, u_center])

        if (not math.isfinite(cx)) or (not math.isfinite(cy)) or (not math.isfinite(cz)):
            self.get_logger().debug(
                f'Center point at (u={u_center}, v={v_center}) is NaN/Inf.'
            )
        else:
            # Publish center point
            center_point_msg = PointStamped()
            center_point_msg.header = msg.header
            center_point_msg.point.x = cx
            center_point_msg.point.y = cy
            center_point_msg.point.z = cz
            self.center_point_pub.publish(center_point_msg)

            # Publish marker at center
            marker = Marker()
            marker.header = msg.header
            marker.ns = 'center_point'
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = cx
            marker.pose.position.y = cy
            marker.pose.position.z = cz
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            self.marker_pub.publish(marker)

        # --- Use your 2D box helper to get window bounds (for debug/consistency) ---
        x1, y1, x2, y2 = segment_2dbox(
            center_point=(u_center, v_center),
            size=(self.window_h, self.window_w),
        )

        # Convert to integer indices and clamp
        u1 = max(0, int(round(x1)))
        v1 = max(0, int(round(y1)))
        u2 = min(width, int(round(x2)))
        v2 = min(height, int(round(y2)))

        if u1 >= u2 or v1 >= v2:
            self.get_logger().debug(
                f'Invalid window after clamping: u1={u1}, u2={u2}, v1={v1}, v2={v2}'
            )
            return

        # --- Extract window (v in [v1, v2), u in [u1, u2)) ---
        window_points = points_2d[v1:v2, u1:u2, :].reshape(-1, 3)

        # Remove points with NaN / Inf
        finite_mask = np.isfinite(window_points).all(axis=1)
        window_points = window_points[finite_mask]

        if window_points.shape[0] == 0:
            self.get_logger().debug(
                f'Window ({u1}:{u2}, {v1}:{v2}) has no valid points.'
            )
        else:
            # --- Publish segmented cloud ---
            seg_cloud = pc2.create_cloud_xyz32(msg.header, window_points.tolist())
            self.segment_cloud_pub.publish(seg_cloud)

        # --- Advance sliding window position for next callback ---
        # Move in u direction first
        self.cur_u0 += self.stride_w
        if self.cur_u0 + self.window_w > width:
            # wrap to next "row" of windows
            self.cur_u0 = 0
            self.cur_v0 += self.stride_h
            if self.cur_v0 + self.window_h > height:
                # wrap back to top
                self.cur_v0 = 0

        # Optional debug log
        self.get_logger().debug(
            f'Window: u[{u1},{u2}), v[{v1},{v2}), center (u={u_center}, v={v_center}), '
            f'points={window_points.shape[0]}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CenterPointFromCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
