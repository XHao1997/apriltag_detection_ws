#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np


class TreeBaseTo3DNode(Node):
    def __init__(self):
        super().__init__('tree_base_to_3d_node_pc')

        self.get_logger().info('TreeBaseTo3DNode (from organized PointCloud2) started.')

        # Parameters
        self.declare_parameter('pointcloud_topic', '/camera/pointcloud2')
        self.declare_parameter('debug', True)

        self.pointcloud_topic = self.get_parameter(
            'pointcloud_topic').get_parameter_value().string_value
        self.debug = self.get_parameter(
            'debug').get_parameter_value().bool_value

        # Latest organized point cloud and its header
        self.cloud_msg = PointCloud2()     # type: PointCloud2
        self.cloud_xyz = None     

        # 1) Organized point cloud (must be aligned to RGB so that (u,v) match)
        self.pc_sub = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.pointcloud_callback,
            10
        )

        # 2) 2D tree base position in RGB image (pixel coordinates u,v)
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

    def pointcloud_callback(self, msg: PointCloud2):
        """
        Store latest organized point cloud and convert to a (H,W,3) numpy array.
        Assumes fields: x, y, z; skip_nans=False to keep the grid shape.
        """
        if msg.height == 1:
            # Not organized: can't index with (v,u)
            self.get_logger().error(
                f'Received UNORGANIZED point cloud on {self.pointcloud_topic} '
                f'(height==1). Need organized cloud for (u,v) lookup.'
            )
            return

        try:
            pts = pc2.read_points_numpy(
                msg,
                field_names=['x', 'y', 'z'],
                skip_nans=False
            )
        except Exception as e:
            self.get_logger().error(f'Failed to read points from PointCloud2: {e}')
            return

        H = msg.height
        W = msg.width
        if pts.shape[0] != H * W:
            self.get_logger().warn(
                f'PointCloud2 size mismatch: N={pts.shape[0]} != H*W={H*W}'
            )
            return

        try:
            xyz = pts.reshape((H, W, 3))
        except ValueError as e:
            self.get_logger().warn(f'Failed to reshape to (H,W,3): {e}')
            return

        self.cloud_msg = msg
        self.cloud_xyz = xyz

        if self.debug:
            self.get_logger().debug(
                f'Updated organized cloud: frame={msg.header.frame_id}, '
                f'H={H}, W={W}'
            )

    def tree_callback(self, tree_msg: PointStamped):
        """
        Convert 2D pixel (u,v) in color image to 3D point
        by indexing the organized point cloud at (v,u).
        """
        if self.cloud_msg is None or self.cloud_xyz is None:
            self.get_logger().warn(
                'No organized point cloud received yet, cannot compute 3D point.'
            )
            return

        # Interpret tree_base_pose.point.x, .y as pixel coordinates (u, v)
        u = int(tree_msg.point.x)
        v = int(tree_msg.point.y)

        H, W, _ = self.cloud_xyz.shape
        if self.debug:
            self.get_logger().info(
                f'tree_callback: pixel (u,v)=({u},{v}), cloud size=(H={H}, W={W})'
            )

        # Boundary check
        if not (0 <= u < W and 0 <= v < H):
            self.get_logger().warn(
                f'tree_base_pose pixel out of range: u={u}, v={v}, width={W}, height={H}'
            )
            return

        # Get XYZ at that pixel
        x, y, z = map(float, self.cloud_xyz[v, u])

        # Check validity (NaN or enormous values)
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            self.get_logger().warn(
                f'Point at (u,v)=({u},{v}) is invalid (NaN or inf), cannot publish 3D point.'
            )
            return

        # Build PointStamped in the cloud frame
        out = PointStamped()
        # Use the cloud's timestamp so it stays consistent with the point cloud
        out.header = self.cloud_msg.header
        out.point.x = x
        out.point.y = y
        out.point.z = z

        self.point_3d_pub.publish(out)

        self.get_logger().info(
            f'Published 3D tree base point from organized cloud: '
            f'pixel=({u},{v}) -> XYZ=({x:.3f}, {y:.3f}, {z:.3f}) '
            f'in frame {out.header.frame_id}'
        )


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
