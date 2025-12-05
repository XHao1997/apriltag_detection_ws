#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener

_current_dir = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.abspath(os.path.join(_current_dir, '..', '..'))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

from soil_task_vision.scripts.utils.cloud_segmentation import (
    transform_cloud_to_frame,
    mean_normal_over_frames,
    arrow_points_from_normal,
    PlaneZStats,
)


class CenterPointFromCloud(Node):
    def __init__(self):
        super().__init__('center_point_from_cloud')

        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("debug", True)

        self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value
        self.debug = self.get_parameter("debug").get_parameter_value().bool_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.use_tf = True

        self.pc_sub = self.create_subscription(PointCloud2, '/camera/pointcloud2', self.cb, 10)
        self.tree_sub = self.create_subscription(PointStamped, '/tree_base_pose', self.tree_cb, 10)

        self.center_pub = self.create_publisher(PointStamped, '/center_point', 10)
        self.center_mk_pub = self.create_publisher(Marker, '/center_point_marker', 10)
        self.seg_pub = self.create_publisher(PointCloud2, '/segment_pointcloud', 10)
        self.normal_mk_pub = self.create_publisher(Marker, '/plane_normal_marker', 10)
        self.window_box_pub = self.create_publisher(Marker, '/window_box_marker', 10)

        self.strip_width = 2
        self.strip_height = 150
        self.num_segments = 3
        self.sample_frames = 1

        # 识别点来自 /tree_base_pose
        self.recog_u = None
        self.recog_v = None

        self.frame_count = 0
        self.seg_buffers = [[] for _ in range(self.num_segments)]

        self.get_logger().info(
            "CenterPointFromCloud: strip split into segments, 5-frame averaging, "
            "pick flattest-2 then closest to base; recog from /tree_base_pose."
        )

    def tree_cb(self, msg: PointStamped):
        u = int(round(msg.point.x))
        v = int(round(msg.point.y))
        self.recog_u = u
        self.recog_v = v
        if self.debug:
            self.get_logger().info(f"Update recog from /tree_base_pose: (u,v)=({u},{v})")

    def _extract_strip(self, pts2d, center_uv, width, height):
        half_w = self.strip_width // 2
        half_h = self.strip_height // 2
        u_c = int(round(center_uv[0]))
        v_c = int(round(center_uv[1]))
        u1 = max(0, u_c - half_w)
        u2 = min(width, u_c + half_w)
        v1 = max(0, v_c - half_h)
        v2 = min(height, v_c + half_h)
        if u1 >= u2 or v1 >= v2:
            return None, (u1, v1, u2, v2)
        strip = pts2d[v1:v2, u1:u2, :]
        return strip, (u1, v1, u2, v2)

    def _segment_bounds(self, bbox):
        u1, v1, u2, v2 = bbox
        h_strip = v2 - v1
        if h_strip <= 0:
            return []
        seg_h = max(1, h_strip // self.num_segments)
        bounds = []
        for i in range(self.num_segments):
            vs = v1 + i * seg_h
            ve = v1 + (i + 1) * seg_h if i < self.num_segments - 1 else v2
            if vs < ve:
                bounds.append((vs, ve))
        return bounds

    def _pick_flattest_segment_from_buffers(self, bbox):
        u1, v1, u2, v2 = bbox
        candidates = []

        for i in range(self.num_segments):
            if not self.seg_buffers[i]:
                continue
            all_pts = np.vstack(self.seg_buffers[i])
            mask = np.isfinite(all_pts).all(axis=1)
            all_pts = all_pts[mask]
            if all_pts.shape[0] < 3:
                continue
            z_vals = all_pts[:, 2]
            z_vals = z_vals[np.isfinite(z_vals)]
            if z_vals.size == 0:
                continue
            z_stats = PlaneZStats(z_values=z_vals)
            score = z_stats.evaluate_flat_level(w_a=2000.0, w_b=200.0)
            centroid = np.mean(all_pts, axis=0)
            dist_to_base = float(np.linalg.norm(centroid))
            candidates.append((i, score, all_pts, centroid, dist_to_base))

        if not candidates:
            return None, None, None, None

        candidates.sort(key=lambda x: x[1])
        top2 = candidates[:2]

        if len(top2) == 1:
            chosen = top2[0]
        else:
            c1, c2 = top2
            chosen = c1 if c1[4] <= c2[4] else c2  # 距离 base 更近

        chosen_idx, chosen_score, chosen_pts, _, _ = chosen

        h_strip = v2 - v1
        seg_h = max(1, h_strip // self.num_segments)
        vs = v1 + chosen_idx * seg_h
        ve = v1 + (chosen_idx + 1) * seg_h if chosen_idx < self.num_segments - 1 else v2
        chosen_bbox = (u1, vs, u2, ve)
        return chosen_idx, chosen_pts, chosen_bbox, chosen_score

    def _publish_window_box(self, header, pts2d, bbox):
        if not self.debug:
            return
        u1, v1, u2, v2 = bbox
        corners_uv = [(u1, v1), (u2 - 1, v1), (u2 - 1, v2 - 1), (u1, v2 - 1)]
        corner_pts = []
        for uu, vv in corners_uv:
            x, y, z = map(float, pts2d[vv, uu])
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                return
            corner_pts.append(Point(x=x, y=y, z=z))
        corner_pts.append(corner_pts[0])
        box = Marker()
        box.header = header
        box.ns = "best_window_box"
        box.id = 0
        box.type = Marker.LINE_STRIP
        box.action = Marker.ADD
        box.scale.x = 0.004
        box.color.r = 1.0
        box.color.g = 0.0
        box.color.b = 0.0
        box.color.a = 1.0
        box.points = corner_pts
        self.window_box_pub.publish(box)

    def _clear_buffers(self):
        self.frame_count = 0
        self.seg_buffers = [[] for _ in range(self.num_segments)]

    def cb(self, msg: PointCloud2):
        if self.recog_u is None or self.recog_v is None:
            if self.debug:
                self.get_logger().warn("No /tree_base_pose yet, skip this frame.")
            return

        cloud = transform_cloud_to_frame(
            self.tf_buffer,
            msg,
            self.target_frame if self.use_tf else None,
            logger=self.get_logger()
        )
        if cloud.height == 1:
            return

        width, height = cloud.width, cloud.height
        if width == 0 or height == 0:
            return

        pts = pc2.read_points_numpy(cloud, field_names=(['x', 'y', 'z']), skip_nans=False)
        if pts.size == 0:
            return
        try:
            pts2d = pts.reshape((height, width, 3))
        except ValueError:
            return

        strip, bbox = self._extract_strip(pts2d, (self.recog_u, self.recog_v), width, height)
        if strip is None:
            return

        bounds = self._segment_bounds(bbox)
        if not bounds:
            return

        for idx, (vs, ve) in enumerate(bounds):
            seg = strip[(vs - bbox[1]):(ve - bbox[1]), :, :].reshape(-1, 3)
            seg = seg[np.isfinite(seg).all(axis=1)]
            if seg.shape[0] >= 3:
                self.seg_buffers[idx].append(seg)

        self.frame_count += 1
        if self.frame_count < self.sample_frames:
            return

        best_idx, best_pts_combined, best_bbox, best_score = self._pick_flattest_segment_from_buffers(bbox)
        if best_idx is None:
            if self.debug:
                self.get_logger().info("No valid segment after 5-frame accumulation.")
            self._clear_buffers()
            return

        try:
            windows = self.seg_buffers[best_idx]
            mean_normal, mean_centroid, stability = mean_normal_over_frames(windows)
        except RuntimeError:
            self._clear_buffers()
            return

        center_msg = PointStamped()
        center_msg.header = cloud.header
        center_msg.point.x = float(mean_centroid[0])
        center_msg.point.y = float(mean_centroid[1])
        center_msg.point.z = float(mean_centroid[2])
        self.center_pub.publish(center_msg)

        if self.debug:
            mk_center = Marker()
            mk_center.header = cloud.header
            mk_center.ns = "best_scan_center"
            mk_center.id = 0
            mk_center.type = Marker.SPHERE
            mk_center.action = Marker.ADD
            mk_center.pose.position.x = center_msg.point.x
            mk_center.pose.position.y = center_msg.point.y
            mk_center.pose.position.z = center_msg.point.z
            mk_center.pose.orientation.w = 1.0
            mk_center.scale.x = mk_center.scale.y = mk_center.scale.z = 0.06
            mk_center.color.r = 1.0
            mk_center.color.g = 0.0
            mk_center.color.b = 0.0
            mk_center.color.a = 1.0
            self.center_mk_pub.publish(mk_center)

            p0, p1 = arrow_points_from_normal(mean_centroid, mean_normal, length=0.25)
            arrow = Marker()
            arrow.header = cloud.header
            arrow.ns = "best_plane_normal"
            arrow.id = 0
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [p0, p1]
            arrow.scale.x = 0.01
            arrow.scale.y = 0.02
            arrow.scale.z = 0.03
            arrow.color.r = 1.0
            arrow.color.g = 0.0
            arrow.color.b = 0.0
            arrow.color.a = 1.0
            self.normal_mk_pub.publish(arrow)

            seg_cloud = pc2.create_cloud_xyz32(cloud.header, best_pts_combined.tolist())
            self.seg_pub.publish(seg_cloud)

            self._publish_window_box(cloud.header, pts2d, best_bbox)

            self.get_logger().info(
                f"[BEST SEG 5-frame] seg_idx={best_idx}, score={best_score:.6f}, "
                f"center=({mean_centroid[0]:.3f},{mean_centroid[1]:.3f},{mean_centroid[2]:.3f}), "
                f"stability={stability:.5f}"
            )

        self._clear_buffers()


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
