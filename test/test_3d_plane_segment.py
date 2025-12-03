#!/usr/bin/env python3
import math
import os
import sys
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from rm_ros_interfaces.srv import GetProbePosition
from message_filters import Subscriber, TimeSynchronizer
# Ensure project root
_current_dir = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.abspath(os.path.join(_current_dir, '..', '..'))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

from apriltag_detection.scripts.utils.cloud_segmentation import (
    transform_cloud_to_frame,
    extract_square_window_points,
    mean_normal_over_frames,
    arrow_points_from_normal,
    compute_scan_direction,
    step_along_direction,
)

class CenterPointFromCloud(Node):
    def __init__(self):
        super().__init__('center_point_from_cloud')
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("detection_2d_topic", "/tree_keypoint")
        self.declare_parameter("detection_3d_topic", "/treetree_base_point_3d")
        self.detection_2d_topic = self.get_parameter("detection_2d_topic").get_parameter_value().string_value
        self.detection_3d_topic = self.get_parameter("detection_3d_topic").get_parameter_value().string_value
        self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value
        # === TF2（可选，把点云转到 base_link） ===
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.use_tf = True

        self.declare_parameter("debug", True)
        self.debug = self.get_parameter("debug").get_parameter_value().bool_value

        # 订阅与发布
        self.pc_sub = self.create_subscription(PointCloud2, '/camera/pointcloud2', self.cb, 10)
        self.detect3d_sub = Subscriber(PointStamped, self.detection_3d_topic)
        self.detect2d_sub = Subscriber(PointStamped, self.detection_2d_topic)
        self.soil_scan_server = self.create_service(GetProbePosition, 'get_probe_position', self.handle_probe_position)
        self.center_pub = self.create_publisher(PointStamped, '/center_point', 10)
        self.center_mk_pub = self.create_publisher(Marker, '/center_point_marker', 10)
        self.seg_pub = self.create_publisher(PointCloud2, '/segment_pointcloud', 10)
        self.normal_mk_pub = self.create_publisher(Marker, '/plane_normal_marker', 10)

        # 识别点与扫描可视化
        self.recog_mk_pub = self.create_publisher(Marker, '/recog_point_marker', 10)
        self.scan_line_pub = self.create_publisher(Marker, '/scan_line_marker', 10)
        self.window_box_pub = self.create_publisher(Marker, '/window_box_marker', 10)
        # self.soil_scan_server = self.create_service(...)s
        # === 扫描配置 ===
        self.window_size = 32        # 正方形窗口（像素）
        self.stride = 8              # 每次前进的像素数
        self.sample_frames = 20       # 同一窗口累积 5 帧

        # 识别点（此处“假定一个识别坐标”）
        self.recog_u = 100.0
        self.recog_v = 100.0

        # 扫描状态（按你要求用 []）
        self.scan_center = []      # 当前窗口中心 (u, v)
        self.scan_dir = []         # 单位方向向量 (du, dv)
        self.win_buffer = []       # 当前窗口累积的 5 帧点
        self.frozen = False        # True 表示当前窗口还没凑满 5 帧

        self.get_logger().info("Square-window scan along line(recog -> bottom-mid) with 5-frame smoothing.")

    def _ensure_scan_init(self, width: int, height: int):
        if self.recog_u is None or self.recog_v is None:
            self.recog_u = width / 2.0
            self.recog_v = height / 2.0

        # 用空列表的“falsy”特性判断是否需要初始化
        if (not self.scan_center) or (not self.scan_dir):
            self.scan_center = [float(self.recog_u), float(self.recog_v)]
            du, dv = compute_scan_direction((self.recog_u, self.recog_v), width, height)
            self.scan_dir = [float(du), float(dv)]
            self.win_buffer.clear()
            self.frozen = True

    def _publish_recog_and_line(self, cloud_header, pts2d, width, height):
        """可视化识别点与识别点->底部中心的 3D 连线"""
        ru, rv = int(round(self.recog_u)), int(round(self.recog_v))
        if not (0 <= ru < width and 0 <= rv < height):
            return

        # 识别点 3D
        rx, ry, rz = map(float, pts2d[rv, ru])
        if math.isfinite(rx) and math.isfinite(ry) and math.isfinite(rz):
            mk = Marker()
            mk.header = cloud_header
            mk.ns, mk.id = "recog_point", 0
            mk.type, mk.action = Marker.SPHERE, Marker.ADD
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = rx, ry, rz
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = mk.scale.z = 0.045
            mk.color.r = 1.0; mk.color.g = 0.0; mk.color.b = 1.0; mk.color.a = 1.0  # 洋红
            self.recog_mk_pub.publish(mk)

            # 底部中心 3D（向上找最近有效深度）
            bm_u = width // 2
            bx = by = bz = float('nan')
            for dv in range(height):
                vv = height - 1 - dv
                bx_, by_, bz_ = map(float, pts2d[vv, bm_u])
                if math.isfinite(bx_) and math.isfinite(by_) and math.isfinite(bz_):
                    bx, by, bz = bx_, by_, bz_
                    break

            # 画线
            if math.isfinite(bx) and math.isfinite(by) and math.isfinite(bz):
                line = Marker()
                line.header = cloud_header
                line.ns, line.id = "recog_to_bottom_mid", 0
                line.type, line.action = Marker.LINE_LIST, Marker.ADD
                line.scale.x = 0.006
                line.color.r = 0.0; line.color.g = 1.0; line.color.b = 0.0; line.color.a = 1.0
                p0 = Point(x=rx, y=ry, z=rz)
                p1 = Point(x=bx, y=by, z=bz)
                line.points = [p0, p1]
                self.scan_line_pub.publish(line)

    def _publish_window_box(self, cloud_header, pts2d, u1, v1, u2, v2):
        """可视化正方形窗口的 3D 线框"""
        corners_uv = [(u1, v1), (u2-1, v1), (u2-1, v2-1), (u1, v2-1)]
        corner_pts = []
        for uu, vv in corners_uv:
            x, y, z = map(float, pts2d[vv, uu])
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                return
            corner_pts.append(Point(x=x, y=y, z=z))
        corner_pts.append(corner_pts[0])  # 闭合

        box = Marker()
        box.header = cloud_header
        box.ns, box.id = "scan_window_box", 0
        box.type, box.action = Marker.LINE_STRIP, Marker.ADD
        box.scale.x = 0.004
        box.color.r = 0.0; box.color.g = 0.7; box.color.b = 1.0; box.color.a = 1.0
        box.points = corner_pts
        self.window_box_pub.publish(box)

    def cb(self, msg: PointCloud2):
        # 1) TF 到统一坐标（pc_utils 内部应使用消息时间）
        cloud = transform_cloud_to_frame(
            self.tf_buffer,
            msg,
            self.target_frame if self.use_tf else None,
            logger=self.get_logger()
        )

        # Organized 检查
        if cloud.height == 1:
            self.get_logger().error(
                "Received UNORGANIZED point cloud (height==1). "
                "This node needs an organized cloud to index with (v,u)."
            )
            return

        width, height = cloud.width, cloud.height
        if width == 0 or height == 0:
            return

        # 2) organized 点云 -> (H, W, 3)
        pts = pc2.read_points_numpy(cloud, field_names=(['x', 'y', 'z']), skip_nans=False)
        if pts.size == 0:
            return
        try:
            pts2d = pts.reshape((height, width, 3))
        except ValueError:
            self.get_logger().warn("Cloud reshape failed (not organized?).")
            return

        # 3) 初始化扫描（识别点与方向）
        self._ensure_scan_init(width, height)

        if self.debug:
            self.get_logger().info(
                f"frame={cloud.header.frame_id} h={height} w={width} "
                f"scan_center={self.scan_center} dir={self.scan_dir}"
            )

        # 可视化识别点与连线
        self._publish_recog_and_line(cloud.header, pts2d, width, height)

        # 4) 提取当前窗口（正方形）
        win_points, (u1, v1, u2, v2) = extract_square_window_points(
            pts2d, self.scan_center, self.window_size, width, height
        )
        if win_points.shape[0] == 0:
            # 当前中心没点 → 试着前进一步；若到头就回到识别点
            new_center, hit_end = step_along_direction(
                self.scan_center, self.scan_dir, self.stride, self.window_size, width, height
            )
            if hit_end:
                self.scan_center = [float(self.recog_u), float(self.recog_v)]
                du, dv = compute_scan_direction((self.recog_u, self.recog_v), width, height)
                self.scan_dir = [float(du), float(dv)]
            else:
                self.scan_center = list(new_center)  # 保持 list
            self.win_buffer.clear()
            self.frozen = True
            return

        # 5) 发布窗口点云 & 窗口线框
        seg = pc2.create_cloud_xyz32(cloud.header, win_points.tolist())
        self.seg_pub.publish(seg)
        self._publish_window_box(cloud.header, pts2d, u1, v1, u2, v2)

        # 6) 发布窗口中心的 3D 点 & marker（可选）
        u_c = int(round(self.scan_center[0]))
        v_c = int(round(self.scan_center[1]))
        cx, cy, cz = map(float, pts2d[v_c, u_c])
        if math.isfinite(cx) and math.isfinite(cy) and math.isfinite(cz):
            pst = PointStamped()
            pst.header = cloud.header
            pst.point.x, pst.point.y, pst.point.z = cx, cy, cz
            self.center_pub.publish(pst)

            mk = Marker()
            mk.header = cloud.header
            mk.ns, mk.id = "scan_center", 0
            mk.type, mk.action = Marker.SPHERE, Marker.ADD
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = cx, cy, cz
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = mk.scale.z = 0.05
            mk.color.r = 1.0; mk.color.g = 1.0; mk.color.b = 0.0; mk.color.a = 1.0
            self.center_mk_pub.publish(mk)

        # 7) 5 帧时间维度平滑：先收齐 5 帧再推进窗口
        self.win_buffer.append(win_points.copy())
        if len(self.win_buffer) < self.sample_frames:
            self.frozen = True
            return

        # 8) 5 帧就绪：分别拟合并求均值法向量
        try:
            mean_normal, mean_centroid, stability = mean_normal_over_frames(self.win_buffer)
        except RuntimeError:
            # 此窗口可用帧不足，向前推进
            self.win_buffer.clear()
            self.frozen = False
        else:
            # 发布法向量箭头
            p0, p1 = arrow_points_from_normal(mean_centroid, mean_normal, length=0.2)
            arrow = Marker()
            arrow.header = cloud.header
            arrow.ns, arrow.id = "plane_normal_5frame", 0
            arrow.type, arrow.action = Marker.ARROW, Marker.ADD
            arrow.points = [p0, p1]
            arrow.scale.x = 0.01; arrow.scale.y = 0.02; arrow.scale.z = 0.03
            arrow.color.r = 0.0; arrow.color.g = 1.0; arrow.color.b = 1.0; arrow.color.a = 1.0
            self.normal_mk_pub.publish(arrow)

            if self.debug:
                self.get_logger().info(
                    f"[5-frame] center=({u_c},{v_c}) normal={mean_normal}, stability={stability:.5f}"
                )
            # 清空以推进
            self.win_buffer.clear()
            self.frozen = False

        # 9) 推进：沿识别点→底中方向走一步；到头则回到识别点
        if not self.frozen:
            new_center, hit_end = step_along_direction(
                self.scan_center, self.scan_dir, self.stride, self.window_size, width, height
            )
            if hit_end:
                # 回到识别点重新扫描
                self.scan_center = [float(self.recog_u), float(self.recog_v)]
                du, dv = compute_scan_direction((self.recog_u, self.recog_v), width, height)
                self.scan_dir = [float(du), float(dv)]
                self.frozen = True
                self.win_buffer.clear()
            else:
                self.scan_center = list(new_center)
                self.frozen = True  # 新窗口开始再次收 5 帧

    def handle_probe_position(self, request, response):
        # Implement the service callback logic here
        return response

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
