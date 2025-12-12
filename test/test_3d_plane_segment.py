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
from tf2_ros import Buffer, TransformListener, TransformBroadcaster

_current_dir = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.abspath(os.path.join(_current_dir, '..', '..'))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

from soil_task_vision.scripts.utils.cloud_segmentation import (
    transform_cloud_to_frame,
    extract_square_window_points,
)


def rotation_matrix_to_quaternion(R: np.ndarray):
    """
    3x3 rotation matrix -> (x,y,z,w) quaternion.
    """
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


class CenterPointFromCloud(Node):
    def __init__(self):
        super().__init__('center_point_from_cloud')

        # 参数
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("debug", True)
        self.declare_parameter("window_size", 10)               # 正方形窗口大小（像素）
        self.declare_parameter("probe_xy_min_threshold", 0.03)   # 3 cm
        self.declare_parameter("probe_xy_max_threshold", 0.05)   # 5 cm
        self.declare_parameter("scan_stride", 2)                 # 每一步沿射线移动的像素数
        self.declare_parameter("min_points_per_window", 5)

        self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value
        self.debug = self.get_parameter("debug").get_parameter_value().bool_value
        self.window_size = self.get_parameter("window_size").get_parameter_value().integer_value

        self.probe_xy_min_threshold = (
            self.get_parameter("probe_xy_min_threshold").get_parameter_value().double_value
        )
        self.probe_xy_max_threshold = (
            self.get_parameter("probe_xy_max_threshold").get_parameter_value().double_value
        )

        self.scan_stride = self.get_parameter("scan_stride").get_parameter_value().integer_value
        self.min_points_per_window = (
            self.get_parameter("min_points_per_window").get_parameter_value().integer_value
        )

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.use_tf = True

        # 订阅
        self.pc_sub = self.create_subscription(
            PointCloud2, '/camera/pointcloud2', self.cb, 10
        )
        self.tree_uv_sub = self.create_subscription(
            PointStamped, '/tree_base_pose', self.tree_uv_cb, 10
        )
        self.tree_base_3d_sub = self.create_subscription(
            PointStamped, '/tree_base_point_3d', self.tree_base_3d_cb, 10
        )

        # 发布
        self.center_pub = self.create_publisher(PointStamped, '/center_point', 10)
        self.center_mk_pub = self.create_publisher(Marker, '/center_point_marker', 10)
        self.soil_probe_pub = self.create_publisher(PointStamped, '/soil_probe_pose', 10)
        self.soil_probe_mk_pub = self.create_publisher(Marker, '/soil_probe_pose_marker', 10)

        # 状态
        self.recog_u = None
        self.recog_v = None
        self.tree_base_3d = None  # np.array([x, y, z])

        self.get_logger().info(
            "CenterPointFromCloud: scan from tree_base UV towards image bottom center; "
            "primary: z_var(window) < z_var(initial) and %.3f m < XY < %.3f m; "
            "fallback: pick window whose XY is closest to %.3f m. "
            "TF /soil_probe_pose: z-down, yaw = atan2(y, x)."
            % (
                self.probe_xy_min_threshold,
                self.probe_xy_max_threshold,
                self.probe_xy_min_threshold,
            )
        )

    # -------- UV 回调 --------
    def tree_uv_cb(self, msg: PointStamped):
        u = int(round(msg.point.x))
        v = int(round(msg.point.y))
        self.recog_u = u
        self.recog_v = v
        if self.debug:
            self.get_logger().info(f"[tree_uv_cb] (u,v)=({u},{v})")

    # -------- 3D 树基 回调 --------
    def tree_base_3d_cb(self, msg: PointStamped):
        self.tree_base_3d = np.array(
            [msg.point.x, msg.point.y, msg.point.z],
            dtype=float
        )
        if self.debug:
            self.get_logger().info(
                "[tree_base_3d_cb] (%.3f, %.3f, %.3f)" %
                (msg.point.x, msg.point.y, msg.point.z)
            )

    # -------- 点云回调 --------
    def cb(self, msg: PointCloud2):
        if self.recog_u is None or self.recog_v is None:
            if self.debug:
                self.get_logger().warn("No /tree_base_pose yet, skip frame.")
            return

        if self.tree_base_3d is None:
            if self.debug:
                self.get_logger().warn("No /tree_base_point_3d yet, skip frame.")
            return

        # 转到 target_frame
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

        pts = pc2.read_points_numpy(
            cloud, field_names=(['x', 'y', 'z']), skip_nans=False
        )
        if pts.size == 0:
            return

        try:
            pts2d = pts.reshape((height, width, 3))
        except ValueError:
            return

        # ---------------- 1) 初始窗口：树基像素附近 ----------------
        init_pts, init_bbox = extract_square_window_points(
            pts2d,
            [self.recog_u, self.recog_v],
            self.window_size,
            width, height
        )
        if init_pts.shape[0] < self.min_points_per_window:
            if self.debug:
                self.get_logger().warn(
                    "Initial window has too few points: %d" % init_pts.shape[0]
                )
            return

        base_z_vals = init_pts[:, 2]
        base_z_var = float(np.var(base_z_vals))
        if self.debug:
            self.get_logger().info(
                "[init window] bbox=%s, z_var=%.6f" % (init_bbox, base_z_var)
            )

        # ---------------- 2) 扫描方向：tree_base 像素 -> 图像底部中心像素 ----------------
        bottom_center_u = width / 2.0
        bottom_center_v = height - 1.0

        start_uv = np.array([float(self.recog_u), float(self.recog_v)], dtype=np.float32)
        end_uv = np.array([bottom_center_u, bottom_center_v], dtype=np.float32)

        direction_uv = end_uv - start_uv
        norm_dir = np.linalg.norm(direction_uv)
        if norm_dir < 1e-6:
            direction_uv = np.array([0.0, 1.0], dtype=np.float32)
        else:
            direction_uv /= norm_dir  # 单位向量

        if self.debug:
            self.get_logger().info(
                "scan direction (UV) from tree_base to bottom-center = (%.3f, %.3f)"
                % (direction_uv[0], direction_uv[1])
            )

        step_px = max(1, self.scan_stride)

        # 对角线长度（像素）作为最大扫描距离
        max_distance = math.sqrt(width * width + height * height)
        max_steps = int(max_distance / step_px) + 2

        # 为了让窗口完整地落在图像内，中心必须在 [half, W-half), [half, H-half)
        half = self.window_size // 2
        u_min, u_max = half, max(half, width - half)
        v_min, v_max = half, max(half, height - half)

        # ---------------- 3) 第一轮：严格模式 ----------------
        best_centroid = None
        best_dist_xy = None
        best_z_var = None
        best_step = None

        for i in range(1, max_steps + 1):
            offset = direction_uv * float(i * step_px)
            u_center = start_uv[0] + offset[0]
            v_center = start_uv[1] + offset[1]

            if not (u_min <= u_center < u_max and v_min <= v_center < v_max):
                if self.debug:
                    self.get_logger().info(
                        "scan hit image border at step %d (u=%.1f, v=%.1f), stop first pass."
                        % (i, u_center, v_center)
                    )
                break

            window_pts, bbox = extract_square_window_points(
                pts2d,
                [u_center, v_center],
                self.window_size,
                width, height
            )
            if window_pts.shape[0] < self.min_points_per_window:
                continue

            z_vals = window_pts[:, 2]
            z_var = float(np.var(z_vals))

            if z_var >= base_z_var:
                if self.debug:
                    self.get_logger().debug(
                        "step %d: z_var=%.6f >= base_z_var=%.6f, skip."
                        % (i, z_var, base_z_var)
                    )
                continue

            centroid = np.mean(window_pts, axis=0)
            dx = centroid[0] - self.tree_base_3d[0]
            dy = centroid[1] - self.tree_base_3d[1]
            dist_xy = math.hypot(dx, dy)

            if not (self.probe_xy_min_threshold < dist_xy < self.probe_xy_max_threshold):
                if self.debug:
                    self.get_logger().debug(
                        "step %d: dist_xy=%.4f not in (%.4f, %.4f), skip."
                        % (
                            i,
                            dist_xy,
                            self.probe_xy_min_threshold,
                            self.probe_xy_max_threshold,
                        )
                    )
                continue

            best_centroid = centroid
            best_dist_xy = dist_xy
            best_z_var = z_var
            best_step = i
            break

        # ---------------- 4) 第二轮：fallback（只看距离，越接近 3cm 越好） ----------------
        if best_centroid is None:
            if self.debug:
                self.get_logger().info(
                    "First pass found no window with z_var < base_z_var and "
                    "(%.3f m < XY < %.3f m), fallback to nearest XY >= %.3f m."
                    % (
                        self.probe_xy_min_threshold,
                        self.probe_xy_max_threshold,
                        self.probe_xy_min_threshold,
                    )
                )

            best2_centroid = None
            best2_dist_xy = None
            best2_step = None
            best2_score = None

            for i in range(1, max_steps + 1):
                offset = direction_uv * float(i * step_px)
                u_center = start_uv[0] + offset[0]
                v_center = start_uv[1] + offset[1]

                if not (u_min <= u_center < u_max and v_min <= v_center < v_max):
                    if self.debug:
                        self.get_logger().info(
                            "fallback scan hit image border at step %d (u=%.1f, v=%.1f), stop."
                            % (i, u_center, v_center)
                        )
                    break

                window_pts, bbox = extract_square_window_points(
                    pts2d,
                    [u_center, v_center],
                    self.window_size,
                    width, height
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
                if self.debug:
                    self.get_logger().info(
                        "fallback: still no window with dist_xy >= %.3f m, give up."
                        % self.probe_xy_min_threshold
                    )
                return

            best_centroid = best2_centroid
            best_dist_xy = best2_dist_xy
            best_step = best2_step
            best_z_var = None  # fallback 不再限制 z_var

        # ---------------- 5) 发布 PointStamped ----------------
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

        # ---------------- 6) 姿态：z 轴朝下，绕 z 轴旋转 atan2(y, x) ----------------
        # yaw 取 soil_probe 在 base_link 下的位置 (fx, fy) 的方位角
        yaw = math.atan2(fy, fx)

        # z 轴朝下
        z_axis = np.array([0.0, 0.0, -1.0], dtype=np.float64)

        # 绕 z 轴的 yaw，定义 x 轴在 XY 平面上的方向
        x_axis = np.array([math.cos(yaw), math.sin(yaw), 0.0], dtype=np.float64)

        # y = z × x，保证右手系
        y_axis = np.cross(z_axis, x_axis)
        ny = np.linalg.norm(y_axis)
        if ny < 1e-6:
            # 极端情况，给个默认 y 轴
            y_axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        else:
            y_axis = y_axis / ny

        # 再正交化一次 x = y × z
        x_axis = np.cross(y_axis, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)

        # 旋转矩阵列向量为 (x_axis, y_axis, z_axis)
        R = np.column_stack((x_axis, y_axis, z_axis))
        qx, qy, qz, qw = rotation_matrix_to_quaternion(R)

        # ---------------- 7) 发布 TF: base_link -> soil_probe_pose ----------------
        tf_msg = TransformStamped()
        tf_msg.header.stamp = cloud.header.stamp
        tf_msg.header.frame_id = self.target_frame      # parent: base_link
        tf_msg.child_frame_id = "soil_probe_pose"       # child: soil_probe_pose

        tf_msg.transform.translation.x = fx
        tf_msg.transform.translation.y = fy
        tf_msg.transform.translation.z = fz

        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf_msg)

        # ---------------- 8) Debug log & markers ----------------
        if self.debug:
            if best_z_var is not None:
                self.get_logger().info(
                    "[SOIL_PROBE primary] step=%d, probe=(%.3f, %.3f, %.3f), "
                    "dist_xy=%.3f (in (%.3f, %.3f)), z_var<base_z_var, yaw=atan2(%.3f, %.3f)=%.3f"
                    % (
                        best_step,
                        fx, fy, fz,
                        best_dist_xy,
                        self.probe_xy_min_threshold,
                        self.probe_xy_max_threshold,
                        fy, fx, yaw,
                    )
                )
            else:
                self.get_logger().info(
                    "[SOIL_PROBE fallback] step=%d, probe=(%.3f, %.3f, %.3f), "
                    "dist_xy=%.3f (>= %.3f, nearest to it), yaw=atan2(%.3f, %.3f)=%.3f"
                    % (
                        best_step,
                        fx, fy, fz,
                        best_dist_xy,
                        self.probe_xy_min_threshold,
                        fy, fx, yaw,
                    )
                )

            # 中心点 marker（红）
            mk_center = Marker()
            mk_center.header = cloud.header
            mk_center.ns = "scan_center"
            mk_center.id = 0
            mk_center.type = Marker.SPHERE
            mk_center.action = Marker.ADD
            mk_center.pose.position.x = fx
            mk_center.pose.position.y = fy
            mk_center.pose.position.z = fz
            mk_center.pose.orientation.w = 1.0
            mk_center.scale.x = mk_center.scale.y = mk_center.scale.z = 0.05
            mk_center.color.r = 1.0
            mk_center.color.g = 0.0
            mk_center.color.b = 0.0
            mk_center.color.a = 1.0
            self.center_mk_pub.publish(mk_center)

            # soil probe marker（绿）
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
