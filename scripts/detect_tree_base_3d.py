#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PointStamped, TransformStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException # type: ignore


def quaternion_to_rot_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
  """(x, y, z, w) -> 3x3 rotation matrix."""
  n = math.sqrt(x * x + y * y + z * z + w * w)
  if n < 1e-9:
    return np.eye(3, dtype=np.float64)
  x /= n
  y /= n
  z /= n
  w /= n

  xx = x * x
  yy = y * y
  zz = z * z
  xy = x * y
  xz = x * z
  yz = y * z
  wx = w * x
  wy = w * y
  wz = w * z

  R = np.array([
      [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),         2.0 * (xz + wy)],
      [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz),   2.0 * (yz - wx)],
      [2.0 * (xz - wy),       2.0 * (yz + wx),         1.0 - 2.0 * (xx + yy)],
  ], dtype=np.float64)
  return R


def rotmat_to_quat(R: np.ndarray):
  """
  3x3 rotation matrix -> (x, y, z, w)
  标准从旋转矩阵到四元数的实现。
  """
  tr = R[0, 0] + R[1, 1] + R[2, 2]

  if tr > 0.0:
    S = math.sqrt(tr + 1.0) * 2.0
    qw = 0.25 * S
    qx = (R[2, 1] - R[1, 2]) / S
    qy = (R[0, 2] - R[2, 0]) / S
    qz = (R[1, 0] - R[0, 1]) / S
  elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
    S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
    qw = (R[2, 1] - R[1, 2]) / S
    qx = 0.25 * S
    qy = (R[0, 1] + R[1, 0]) / S
    qz = (R[0, 2] + R[2, 0]) / S
  elif R[1, 1] > R[2, 2]:
    S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
    qw = (R[0, 2] - R[2, 0]) / S
    qx = (R[0, 1] + R[1, 0]) / S
    qy = 0.25 * S
    qz = (R[1, 2] + R[2, 1]) / S
  else:
    S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
    qw = (R[1, 0] - R[0, 1]) / S
    qx = (R[0, 2] + R[2, 0]) / S
    qy = (R[1, 2] + R[2, 1]) / S
    qz = 0.25 * S

  return float(qx), float(qy), float(qz), float(qw)


class TreeBaseTo3DNode(Node):
  def __init__(self):
    super().__init__('tree_base_to_3d_node_pc')

    self.get_logger().info('TreeBaseTo3DNode (from organized PointCloud2) with TF started.')

    # Parameters
    self.declare_parameter('pointcloud_topic', '/camera/pointcloud2')
    self.declare_parameter('parent_frame', 'base_link')     # 希望树基在谁下面表达（通常 base_link）
    self.declare_parameter('tree_frame', 'tree_base_3d')    # 子坐标系名称
    self.declare_parameter('z_median_threshold', 0.02)      # z 去噪阈值（m）
    self.declare_parameter('debug', True)

    self.pointcloud_topic = self.get_parameter(
        'pointcloud_topic').get_parameter_value().string_value
    self.parent_frame = self.get_parameter(
        'parent_frame').get_parameter_value().string_value
    self.tree_frame = self.get_parameter(
        'tree_frame').get_parameter_value().string_value
    self.z_median_threshold = self.get_parameter(
        'z_median_threshold').get_parameter_value().double_value
    self.debug = self.get_parameter(
        'debug').get_parameter_value().bool_value

    # 最新有序点云及其 xyz
    self.cloud_msg = None          # type: PointCloud2 | None
    self.cloud_xyz = None          # type: np.ndarray | None

    # TF: 用来从点云帧变换到 parent_frame（base_link）
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
    self.tf_broadcaster = TransformBroadcaster(self)

    # 订阅点云（要求 height>1 的 organized cloud）
    self.pc_sub = self.create_subscription(
        PointCloud2,
        self.pointcloud_topic,
        self.pointcloud_callback,
        10
    )

    # 订阅 2D 树基像素坐标
    self.tree_sub = self.create_subscription(
        PointStamped,
        '/tree_base_pose',
        self.tree_callback,
        10
    )

    # 发布 tree_base_point_3d（在 parent_frame 下，比如 base_link）
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
    Convert 2D pixel (u,v) in color image to 3D point using the organized point cloud.
    使用 5 邻域 (center/up/down/left/right)，按 z 中值做简单去噪后取均值。
    然后通过 TF 将该点从 pointcloud frame 转到 parent_frame（例如 base_link），
    并在 parent_frame 下假设树坐标系 z 轴朝下（绕 x 轴 180°），发布 TF。
    """
    if self.cloud_msg is None or self.cloud_xyz is None:
      self.get_logger().warn(
          'No organized point cloud received yet, cannot compute 3D point.'
      )
      return

    # 像素坐标
    u_center = int(tree_msg.point.x)
    v_center = int(tree_msg.point.y)

    H, W, _ = self.cloud_xyz.shape
    if self.debug:
      self.get_logger().info(
          f'tree_callback: center pixel (u,v)=({u_center},{v_center}), '
          f'cloud size=(H={H}, W={W}), cloud_frame={self.cloud_msg.header.frame_id}'
      )

    # 5-neighborhood: center + four neighbors
    neighbor_offsets = [
        (0, 0),   # center
        (0, -1),  # up
        (0, 1),   # down
        (-1, 0),  # left
        (1, 0),   # right
    ]

    samples = []

    for du, dv in neighbor_offsets:
      u = u_center + du
      v = v_center + dv

      # Boundary check
      if not (0 <= u < W and 0 <= v < H):
        continue

      x, y, z = map(float, self.cloud_xyz[v, u])

      # Filter NaN/Inf
      if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
        continue

      samples.append([x, y, z])

    if not samples:
      self.get_logger().warn(
          f'No valid points found in 5-neighborhood around (u,v)=({u_center},{v_center}).'
      )
      return

    pts = np.asarray(samples, dtype=float)

    # z 去噪: 用中值 + 阈值
    z_vals = pts[:, 2]
    z_med = float(np.median(z_vals))
    abs_dev = np.abs(z_vals - z_med)
    z_threshold = self.z_median_threshold  # 例如 0.02 m

    mask = abs_dev <= z_threshold
    filtered = pts[mask]

    if filtered.shape[0] == 0:
      # 全被过滤就退回所有样本
      filtered = pts

    mean_xyz = filtered.mean(axis=0)
    x_c, y_c, z_c = map(float, mean_xyz)

    cloud_frame = self.cloud_msg.header.frame_id

    # 通过 TF: parent_frame <- cloud_frame
    try:
      tf_pc = self.tf_buffer.lookup_transform(
          self.parent_frame,   # target
          cloud_frame,         # source
          Time()               # latest
      )
    except TransformException as ex:
      self.get_logger().warn(
          f'Failed to lookup TF {self.parent_frame} <- {cloud_frame}: {ex}. '
          f'Cannot publish tree_base_3d in {self.parent_frame}.'
      )
      return

    R_pc = quaternion_to_rot_matrix(
        tf_pc.transform.rotation.x,
        tf_pc.transform.rotation.y,
        tf_pc.transform.rotation.z,
        tf_pc.transform.rotation.w,
    )
    t_pc = np.array([
        tf_pc.transform.translation.x,
        tf_pc.transform.translation.y,
        tf_pc.transform.translation.z,
    ], dtype=np.float64)

    p_cloud = np.array([x_c, y_c, z_c], dtype=np.float64)
    p_parent = R_pc @ p_cloud + t_pc

    x_b, y_b, z_b = map(float, p_parent)

    # 在 parent_frame (base_link) 下发布 PointStamped
    out = PointStamped()
    out.header.stamp = self.cloud_msg.header.stamp
    out.header.frame_id = self.parent_frame
    out.point.x = x_b
    out.point.y = y_b
    out.point.z = z_b
    self.point_3d_pub.publish(out)

    # ----------------- 发布 TF: parent_frame -> tree_base_3d -----------------
    # 在 parent_frame 下假设树坐标系 z 轴朝下：绕 X 轴 180°。
    R_btree = np.array([
        [1.0,  0.0,  0.0],
        [0.0, -1.0,  0.0],
        [0.0,  0.0, -1.0],
    ], dtype=np.float64)

    qx, qy, qz, qw = rotmat_to_quat(R_btree)

    t = TransformStamped()
    t.header.stamp = self.cloud_msg.header.stamp
    t.header.frame_id = self.parent_frame
    t.child_frame_id = self.tree_frame

    t.transform.translation.x = x_b
    t.transform.translation.y = y_b
    t.transform.translation.z = z_b

    t.transform.rotation.x = qx
    t.transform.rotation.y = qy
    t.transform.rotation.z = qz
    t.transform.rotation.w = qw

    self.tf_broadcaster.sendTransform(t)

    if self.debug:
      self.get_logger().info(
          'Published tree base 3D point & TF:\n'
          f'  cloud_frame={cloud_frame}, parent_frame={self.parent_frame}\n'
          f'  pixel=({u_center},{v_center}), '
          f'valid_samples={pts.shape[0]}, used_samples={filtered.shape[0]}\n'
          f'  XYZ_parent=({x_b:.3f}, {y_b:.3f}, {z_b:.3f}), '
          f'orientation=z-down in {self.parent_frame}'
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
