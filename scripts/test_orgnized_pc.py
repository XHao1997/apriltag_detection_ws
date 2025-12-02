#!/usr/bin/env python3
import math

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker


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

        # Publishes the 3D point (for debugging / other nodes)
        self.center_point_pub = self.create_publisher(
            PointStamped,
            '/center_point',
            10,
        )

        # Publishes a Marker so you can see it in RViz
        self.marker_pub = self.create_publisher(
            Marker,
            '/center_point_marker',
            10,
        )

        self.get_logger().info('CenterPointFromCloud node started.')

    def pointcloud_callback(self, msg: PointCloud2):
        # Check organized cloud
        if msg.height == 1:
            self.get_logger().warn_once(
                'PointCloud2 is unorganized (height=1). '
                'Center pixel concept may not be valid.'
            )

        width = msg.width
        height = msg.height

        if width == 0 or height == 0:
            return

        # Choose pixel (you can change to width//2, height//2)
        u = 424
        v = 240

        if u < 0 or u >= width or v < 0 or v >= height:
            self.get_logger().warn_once(
                f'(u, v)=({u}, {v}) is out of bounds for cloud size {width}x{height}'
            )
            return

        # Read the 3D point at (u, v)
        points = pc2.read_points_numpy(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=False,
        )

        if points.size == 0:
            self.get_logger().warn(
                f'No point at (u={u}, v={v}) in PointCloud2'
            )
            return
        print(points)
        # Only one point requested → shape (1, 3)
        x, y, z = map(float, points[v*width+u])
        print(u, v)
        print(x, y, z)

        # NaN / Inf check
        if (not math.isfinite(x)) or (not math.isfinite(y)) or (not math.isfinite(z)):
            self.get_logger().warn(
                f'Point at (u={u}, v={v}) is invalid or NaN: ({x}, {y}, {z})'
            )
            return

        # ----- Publish PointStamped -----
        center_point_msg = PointStamped()
        center_point_msg.header = msg.header
        center_point_msg.point.x = x
        center_point_msg.point.y = y
        center_point_msg.point.z = z
        self.center_point_pub.publish(center_point_msg)

        # ----- Publish Marker (sphere) -----
        marker = Marker()
        marker.header = msg.header
        marker.ns = 'center_point'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0

        # Sphere size in meters (0.2 is quite big; 0.02 is more typical)
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # Red sphere, fully opaque
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

        self.get_logger().debug(
            f'Center point (u={u}, v={v}) -> ({x:.3f}, {y:.3f}, {z:.3f})'
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
