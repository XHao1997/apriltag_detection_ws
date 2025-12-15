#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, TransformStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool

from tf2_ros import Buffer, TransformListener, TransformBroadcaster

_current_dir = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.abspath(os.path.join(_current_dir, '..', '..'))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

from utils.cloud_segmentation import (
    transform_cloud_to_frame,
    extract_square_window_points,
)


def rotation_matrix_to_quaternion(R: np.ndarray):
    """3x3 rotation matrix -> (x,y,z,w) quaternion."""
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]

    tr = m00 + m11 + m22

    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S
        qz = (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
    elif m11 > m22:
        S = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
    else:
        S = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S

    return float(qx), float(qy), float(qz), float(qw)


def stamp_to_sec(stamp) -> float:
    """builtin_interfaces/Time -> float seconds"""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class CenterPointFromCloud(Node):
    def __init__(self):
        super().__init__('center_point_from_cloud')

        # ---------------- Parameters ----------------
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("debug", True)

        self.declare_parameter("window_size", 10)                 # 正方形窗口大小（像素）
        self.declare_parameter("probe_xy_min_threshold", 0.05)    # 3 cm
        self.declare_parameter("probe_xy_max_threshold", 0.10)    # 5 cm
        self.declare_parameter("scan_stride", 2)                  # 每一步沿射线移动的像素数
        self.declare_parameter("min_points_per_window", 5)

        # 时间同步：tree_base_pose vs tree_base_point_3d
        self.declare_parameter("sync_tolerance_sec", 0.6)

        # 输入“新鲜度”：相对点云 stamp，超过这个就认为没识别（立刻停止）
        self.declare_parameter("max_input_age_to_cloud_sec", 0.8)

        # 不持续更新：仅当 detection stamp 变化才发布一次
        self.declare_parameter("publish_on_new_detection_only", True)

        # 是否发布 valid 标志（强烈建议下游用它立刻停止）
        self.declare_parameter("publish_valid_flag", True)

        self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value
        self.debug = self.get_parameter("debug").get_parameter_value().bool_value

        self.window_size = self.get_parameter("window_size").get_parameter_value().integer_value
        self.probe_xy_min_threshold = self.get_parameter("probe_xy_min_threshold").get_parameter_value().double_value
        self.probe_xy_max_threshold = self.get_parameter("probe_xy_max_threshold").get_parameter_value().double_value
        self.scan_stride = self.get_parameter("scan_stride").get_parameter_value().integer_value
        self.min_points_per_window = self.get_parameter("min_points_per_window").get_parameter_value().integer_value

        self.sync_tolerance_sec = self.get_parameter("sync_tolerance_sec").get_parameter_value().double_value
        self.max_input_age_to_cloud_sec = self.get_parameter("max_input_age_to_cloud_sec").get_parameter_value().double_value
        self.publish_on_new_detection_only = self.get_parameter("publish_on_new_detection_only").get_parameter_value().bool_value
        self.publish_valid_flag = self.get_parameter("publish_valid_flag").get_parameter_value().bool_value

        # ---------------- TF ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.use_tf = True

        # ---------------- Subscribers ----------------
        self.pc_sub = self.create_subscription(PointCloud2, '/camera/pointcloud2', self.cb, 10)
        self.tree_uv_sub = self.create_subscription(PointStamped, '/tree_base_pose', self.tree_uv_cb, 10)
        self.tree_base_3d_sub = self.create_subscription(PointStamped, '/tree_base_point_3d', self.tree_base_3d_cb, 10)

        # ---------------- Publishers ----------------
        self.center_pub = self.create_publisher(PointStamped, '/center_point', 10)
        self.center_mk_pub = self.create_publisher(Marker, '/center_point_marker', 10)
        self.soil_probe_pub = self.create_publisher(PointStamped, '/soil_probe_pose', 10)
        self.soil_probe_mk_pub = self.create_publisher(Marker, '/soil_probe_pose_marker', 10)

        self.valid_pub = self.create_publisher(Bool, '/soil_probe_valid', 10) if self.publish_valid_flag else None

        # ---------------- State ----------------
        self.recog_u = None
        self.recog_v = None
        self.tree_uv_stamp = None

        self.tree_base_3d = None
        self.tree_base_3d_stamp = None

        # 用于“只在新识别时发布一次”
        self._last_publish_key = None  # (uv_stamp_sec, base3d_stamp_sec)

        self.get_logger().info(
            "CenterPointFromCloud:\n"
            f"  target_frame={self.target_frame}\n"
            f"  sync_tolerance_sec={self.sync_tolerance_sec}\n"
            f"  max_input_age_to_cloud_sec={self.max_input_age_to_cloud_sec}\n"
            f"  publish_on_new_detection_only={self.publish_on_new_detection_only}\n"
            "  TF soil_probe_pose: z-down, yaw=atan2(y,x)\n"
            "  If invalid -> publish /soil_probe_valid=false and stop publishing TF."
        )

    # ---------------- Helpers ----------------
    def publish_valid(self, is_valid: bool):
        if self.valid_pub is None:
            return
        msg = Bool()
        msg.data = bool(is_valid)
        self.valid_pub.publish(msg)

    def tree_base_3d_valid(self) -> bool:
        if self.tree_base_3d is None or self.tree_base_3d_stamp is None:
            return False
        if not isinstance(self.tree_base_3d, np.ndarray) or self.tree_base_3d.shape != (3,):
            return False
        return bool(np.all(np.isfinite(self.tree_base_3d)))

    # ---------------- Callbacks ----------------
    def tree_uv_cb(self, msg: PointStamped):
        self.recog_u = int(round(msg.point.x))
        self.recog_v = int(round(msg.point.y))
        self.tree_uv_stamp = msg.header.stamp
        if self.debug:
            self.get_logger().info(
                f"[tree_uv_cb] (u,v)=({self.recog_u},{self.recog_v}), stamp={stamp_to_sec(self.tree_uv_stamp):.3f}"
            )

    def tree_base_3d_cb(self, msg: PointStamped):
        self.tree_base_3d = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)
        self.tree_base_3d_stamp = msg.header.stamp
        if self.debug:
            self.get_logger().info(
                "[tree_base_3d_cb] (%.3f, %.3f, %.3f), stamp=%.3f"
                % (msg.point.x, msg.point.y, msg.point.z, stamp_to_sec(self.tree_base_3d_stamp))
            )

    def cb(self, msg: PointCloud2):
        # 任何无效输入 -> 立刻停止（valid=false + return，不发布 TF）
        if self.recog_u is None or self.recog_v is None or self.tree_uv_stamp is None:
            self.publish_valid(False)
            return

        if not self.tree_base_3d_valid():
            self.publish_valid(False)
            return

        # tree_base_pose vs tree_base_point_3d 时间戳同步
        t_uv = stamp_to_sec(self.tree_uv_stamp)
        t_3d = stamp_to_sec(self.tree_base_3d_stamp)
        if abs(t_uv - t_3d) > self.sync_tolerance_sec:
            if self.debug:
                self.get_logger().warn(
                    f"Skip: uv/base3d not synced |{t_uv:.3f}-{t_3d:.3f}|>{self.sync_tolerance_sec:.3f}"
                )
            self.publish_valid(False)
            return

        # 相对点云 stamp 的“新鲜度”（没识别到就会立刻 stale）
        t_cloud = stamp_to_sec(msg.header.stamp)
        if (t_cloud - t_uv) > self.max_input_age_to_cloud_sec or (t_cloud - t_3d) > self.max_input_age_to_cloud_sec:
            if self.debug:
                self.get_logger().warn(
                    f"Skip: input stale wrt cloud. "
                    f"(cloud-uv)={(t_cloud-t_uv):.3f}, (cloud-3d)={(t_cloud-t_3d):.3f}, "
                    f"limit={self.max_input_age_to_cloud_sec:.3f}"
                )
            self.publish_valid(False)
            return

        # 不持续更新：只在新 detection 来时才处理一次
        publish_key = (t_uv, t_3d)
        if self.publish_on_new_detection_only and self._last_publish_key == publish_key:
            return  # 不重复发布，不更新 TF
        # 注意：只有成功发布后才更新 _last_publish_key

        # 点云转到 target_frame
        cloud = transform_cloud_to_frame(
            self.tf_buffer,
            msg,
            self.target_frame if self.use_tf else None,
            logger=self.get_logger()
        )
        if cloud.height == 1:
            self.publish_valid(False)
            return

        width, height = cloud.width, cloud.height
        if width == 0 or height == 0:
            self.publish_valid(False)
            return

        pts = pc2.read_points_numpy(cloud, field_names=(['x', 'y', 'z']), skip_nans=False)
        if pts.size == 0:
            self.publish_valid(False)
            return

        try:
            pts2d = pts.reshape((height, width, 3))
        except ValueError:
            self.publish_valid(False)
            return

        # 1) 初始窗口：树基像素附近
        init_pts, init_bbox = extract_square_window_points(
            pts2d, [self.recog_u, self.recog_v], self.window_size, width, height
        )
        if init_pts.shape[0] < self.min_points_per_window:
            self.publish_valid(False)
            return
        base_z_var = float(np.var(init_pts[:, 2]))

        # 2) 扫描方向：tree_base 像素 -> 图像底部中心像素
        bottom_center_u = width / 2.0
        bottom_center_v = height - 1.0
        start_uv = np.array([float(self.recog_u), float(self.recog_v)], dtype=np.float32)
        end_uv = np.array([bottom_center_u, bottom_center_v], dtype=np.float32)

        direction_uv = end_uv - start_uv
        norm_dir = float(np.linalg.norm(direction_uv))
        if norm_dir < 1e-6:
            direction_uv = np.array([0.0, 1.0], dtype=np.float32)
        else:
            direction_uv = direction_uv / norm_dir

        step_px = max(1, int(self.scan_stride))
        max_distance = math.sqrt(width * width + height * height)
        max_steps = int(max_distance / step_px) + 2

        half = self.window_size // 2
        u_min, u_max = half, max(half, width - half)
        v_min, v_max = half, max(half, height - half)

        best_centroid = None
        best_dist_xy = None
        best_z_var = None
        best_step = None

        # 3) 第一轮：严格模式（z_var 更平 + 3~5cm）
        for i in range(1, max_steps + 1):
            offset = direction_uv * float(i * step_px)
            u_center = start_uv[0] + offset[0]
            v_center = start_uv[1] + offset[1]

            if not (u_min <= u_center < u_max and v_min <= v_center < v_max):
                break

            window_pts, _ = extract_square_window_points(
                pts2d, [u_center, v_center], self.window_size, width, height
            )
            if window_pts.shape[0] < self.min_points_per_window:
                continue

            z_var = float(np.var(window_pts[:, 2]))
            if z_var >= base_z_var:
                continue

            centroid = np.mean(window_pts, axis=0)
            dx = centroid[0] - self.tree_base_3d[0]
            dy = centroid[1] - self.tree_base_3d[1]
            dist_xy = math.hypot(dx, dy)

            if not (self.probe_xy_min_threshold < dist_xy < self.probe_xy_max_threshold):
                continue

            best_centroid = centroid
            best_dist_xy = dist_xy
            best_z_var = z_var
            best_step = i
            break

        # 4) fallback：只看距离 >= 3cm，越接近 3cm 越好
        if best_centroid is None:
            best2_centroid = None
            best2_dist_xy = None
            best2_step = None
            best2_score = None

            for i in range(1, max_steps + 1):
                offset = direction_uv * float(i * step_px)
                u_center = start_uv[0] + offset[0]
                v_center = start_uv[1] + offset[1]

                if not (u_min <= u_center < u_max and v_min <= v_center < v_max):
                    break

                window_pts, _ = extract_square_window_points(
                    pts2d, [u_center, v_center], self.window_size, width, height
                )
                if window_pts.shape[0] < self.min_points_per_window:
                    continue

                centroid = np.mean(window_pts, axis=0)
                dx = centroid[0] - self.tree_base_3d[0]
                dy = centroid[1] - self.tree_base_3d[1]
                dist_xy = math.hypot(dx, dy)

                if dist_xy < self.probe_xy_min_threshold:
                    continue

                score = abs(dist_xy - self.probe_xy_min_threshold)
                if (best2_centroid is None) or (score < best2_score):
                    best2_centroid = centroid
                    best2_dist_xy = dist_xy
                    best2_step = i
                    best2_score = score

            if best2_centroid is None:
                self.publish_valid(False)
                return

            best_centroid = best2_centroid
            best_dist_xy = best2_dist_xy
            best_step = best2_step
            best_z_var = None

        # 5) 发布 PointStamped / TF
        fx, fy, fz = float(best_centroid[0]), float(best_centroid[1]), float(best_centroid[2])

        center_msg = PointStamped()
        center_msg.header = cloud.header
        center_msg.point.x = fx
        center_msg.point.y = fy
        center_msg.point.z = fz
        self.center_pub.publish(center_msg)

        soil_msg = PointStamped()
        soil_msg.header = cloud.header
        soil_msg.point.x = fx
        soil_msg.point.y = fy
        soil_msg.point.z = fz
        self.soil_probe_pub.publish(soil_msg)

        # 姿态：z 轴朝下；yaw = atan2(y, x) 绕 z 轴旋转
        yaw = math.atan2(fy, fx)+np.pi

        z_axis = np.array([0.0, 0.0, -1.0], dtype=np.float64)
        x_axis = np.array([math.cos(yaw), math.sin(yaw), 0.0], dtype=np.float64)

        y_axis = np.cross(z_axis, x_axis)
        ny = float(np.linalg.norm(y_axis))
        if ny < 1e-6:
            y_axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        else:
            y_axis = y_axis / ny

        x_axis = np.cross(y_axis, z_axis)
        x_axis = x_axis / float(np.linalg.norm(x_axis))

        R = np.column_stack((x_axis, y_axis, z_axis))
        qx, qy, qz, qw = rotation_matrix_to_quaternion(R)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = cloud.header.stamp
        tf_msg.header.frame_id = self.target_frame
        tf_msg.child_frame_id = "soil_probe_pose"

        tf_msg.transform.translation.x = fx
        tf_msg.transform.translation.y = fy
        tf_msg.transform.translation.z = fz
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf_msg)

        # 成功：valid=true + 更新 last key（从此不再持续更新）
        self.publish_valid(True)
        self._last_publish_key = publish_key

        if self.debug:
            if best_z_var is not None:
                self.get_logger().info(
                    f"[SOIL_PROBE primary] step={best_step}, p=({fx:.3f},{fy:.3f},{fz:.3f}), "
                    f"dist_xy={best_dist_xy:.3f}, yaw={yaw:.3f}"
                )
            else:
                self.get_logger().info(
                    f"[SOIL_PROBE fallback] step={best_step}, p=({fx:.3f},{fy:.3f},{fz:.3f}), "
                    f"dist_xy={best_dist_xy:.3f}, yaw={yaw:.3f}"
                )

            mk_probe = Marker()
            mk_probe.header = cloud.header
            mk_probe.ns = "soil_probe"
            mk_probe.id = 0
            mk_probe.type = Marker.SPHERE
            mk_probe.action = Marker.ADD
            mk_probe.pose.position.x = fx
            mk_probe.pose.position.y = fy
            mk_probe.pose.position.z = fz
            mk_probe.pose.orientation.w = 1.0
            mk_probe.scale.x = mk_probe.scale.y = mk_probe.scale.z = 0.07
            mk_probe.color.r = 0.0
            mk_probe.color.g = 1.0
            mk_probe.color.b = 0.0
            mk_probe.color.a = 1.0
            self.soil_probe_mk_pub.publish(mk_probe)


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
