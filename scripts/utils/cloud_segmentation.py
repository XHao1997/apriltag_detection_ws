# apriltag_detection/scripts/utils/pc_utils.py
import numpy as np
from typing import List, Tuple, Optional
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformException
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud, transform_points
import rclpy.time


# ---------- 平面拟合（最小二乘 / SVD） ----------
def fit_plane_least_squares(points: np.ndarray):
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must be (N,3)")
    if points.shape[0] < 3:
        raise ValueError("need >=3 points")
    centroid = points.mean(axis=0)
    pts_centered = points - centroid
    _, _, vh = np.linalg.svd(pts_centered, full_matrices=False)
    normal = vh[-1]
    nrm = np.linalg.norm(normal)
    if nrm < 1e-6:
        raise ValueError("degenerate plane")
    normal = normal / nrm
    d = -normal.dot(centroid)
    return normal, d, centroid
def to_xyz_float32(cloud_in: PointCloud2) -> PointCloud2:
    """
    把输入 PointCloud2 转成只含 xyz 的 float32 点云。
    如果原始点云是有序的 (height>1)，会保留它的 height/width（顺序不变）。
    """
    from sensor_msgs_py import point_cloud2 as pc2
    import numpy as np

    H = cloud_in.height
    W = cloud_in.width

    # 1) 读出 xyz 点（按原始顺序，skip_nans=False 保证数量=H*W）
    pts = pc2.read_points_numpy(
        cloud_in,
        field_names=(['x', 'y', 'z']),
        skip_nans=False,
    )  # shape: (N,3)

    # 2) 确保是 float32
    pts = pts.astype(np.float32)

    # 3) 用 create_cloud_xyz32 重新打包
    header = cloud_in.header
    cloud_out = pc2.create_cloud_xyz32(header, pts)
    cloud_out.height = H
    cloud_out.width = W
    cloud_out.row_step = cloud_out.point_step * W
    # 你有 NaN 的话可以设 is_dense=False
    cloud_out.is_dense = False

    return cloud_out

# ---------- TF 转换到目标坐标系 ----------
def transform_cloud_to_frame(
    tf_buffer: Buffer,
    cloud_msg: PointCloud2,
    target_frame: Optional[str],
    logger=None
) -> PointCloud2:
    """
    手写 TF 变换：cloud_msg.header.frame_id -> target_frame

    - 只用 xyz 三个字段做变换
    - 保留点的顺序
    - 如果原始点云是有序的（height>1），会恢复 height/width，方便你 reshape 成 (H,W,3)
    """
    if not target_frame:
        return cloud_msg

    try:
        # target_frame ← source_frame 的变换
        tf = tf_buffer.lookup_transform(
            target_frame,
            cloud_msg.header.frame_id,
            rclpy.time.Time()
        )
    except TransformException as ex:
        if logger:
            logger.warn(
                f"TF transform {cloud_msg.header.frame_id} -> {target_frame} failed: {ex}. Use original frame."
            )
        return cloud_msg

    # 原始尺寸（判断是否有序点云用）
    H = cloud_msg.height
    W = cloud_msg.width

    # 1) 读出 xyz（按原始顺序）
    pts = pc2.read_points_numpy(
        cloud_msg,
        field_names=(['x', 'y', 'z']),
        skip_nans=False,
    )  # shape: (N,3)

    if pts.size == 0:
        # 空点云：只改 frame_id
        out = PointCloud2()
        out.header = cloud_msg.header
        out.header.frame_id = target_frame
        return out

    # 2) 构造旋转矩阵和平移向量
    t = tf.transform.translation
    q = tf.transform.rotation
    R = quaternion_to_rot_matrix(q.x, q.y, q.z, q.w)          # 3x3
    t_vec = np.array([t.x, t.y, t.z], dtype=np.float64)       # (3,)

    # 3) 应用变换：p' = R * p + t
    pts = pts.astype(np.float64)
    pts_tf = (R @ pts.T).T + t_vec        # (N,3)
    pts_tf = pts_tf.astype(np.float32)

    # 4) 打包成新的 PointCloud2（只含 xyz）
    header = cloud_msg.header
    header.frame_id = target_frame
    cloud_out = pc2.create_cloud_xyz32(header, pts_tf)

    # 5) 如果原来是 organized cloud，恢复 height / width / row_step
    if H > 1 and pts_tf.shape[0] == H * W:
        cloud_out.height = H
        cloud_out.width = W
        cloud_out.row_step = cloud_out.point_step * W
        cloud_out.is_dense = False  # 有 NaN 时最好设 False

    return cloud_out

# ---------- 取窗口内点（正方形） ----------
def extract_square_window_points(
    points_2d: np.ndarray,
    center_uv: list,
    window_size: int,
    width: int, height: int
) -> Tuple[np.ndarray, Tuple[int, int, int, int]]:
    half = window_size // 2
    u_c = int(round(center_uv[0]))
    v_c = int(round(center_uv[1]))
    u1 = max(0, u_c - half)
    v1 = max(0, v_c - half)
    u2 = min(width,  u_c + half)
    v2 = min(height, v_c + half)
    win = points_2d[v1:v2, u1:u2, :].reshape(-1, 3)
    win = win[np.isfinite(win).all(axis=1)]
    return win, (u1, v1, u2, v2)


# ---------- 连续多帧：每帧拟合并求均值 ----------
def mean_normal_over_frames(windows: List[np.ndarray]) -> Tuple[np.ndarray, np.ndarray, np.float32]:
    normals, cents = [], []
    for pts in windows:
        if pts.shape[0] < 3:
            continue
        n_i, _, c_i = fit_plane_least_squares(pts)
        normals.append(n_i)
        cents.append(c_i)
    if not normals:
        raise RuntimeError("No valid frames for plane fitting")
    normals = np.vstack(normals)
    cents   = np.vstack(cents)
    mean_normal = normals.mean(axis=0)
    mean_normal = mean_normal / np.linalg.norm(mean_normal)
    mean_centroid = cents.mean(axis=0)
    stability = np.float32(np.linalg.norm(normals.std(axis=0)))
    return mean_normal, mean_centroid, stability


# ---------- 箭头两端点 ----------
def arrow_points_from_normal(centroid: np.ndarray, normal: np.ndarray, length: float = 0.2) -> Tuple[Point, Point]:
    start = centroid.astype(np.float32)
    end   = (centroid + normal * length)
    p0 = Point(x=float(start[0]), y=float(start[1]), z=float(start[2]))
    p1 = Point(x=float(end[0]),   y=float(end[1]),   z=float(end[2]))
    return p0, p1


# ---------- 识别点 → 图像底部中点 的扫描方向 ----------
def compute_scan_direction(
    recog_uv: Tuple[int, int],
    width: int, height: int
) -> np.ndarray:
    bottom_mid = np.array([width / 2.0, height - 1.0], dtype=np.float32)
    recog = np.array(recog_uv, dtype=np.float32)
    v = bottom_mid - recog
    n = np.linalg.norm(v)
    if n < 1e-6:
        # 识别点刚好在底中，默认向下
        return np.array([0.0, 1.0], dtype=np.float32)
        print("recg locate at mid")
    return v / n


# ---------- 沿方向推进窗口中心 ----------
def step_along_direction(
    center_uv: list,
    direction_uv: list,
    stride: int,
    window_size: int,
    width: int, height: int
) -> Tuple[Tuple[np.float32, np.float32], bool]:
    """
    沿给定方向把窗口中心推进 stride 像素。
    返回：(new_center_uv, hit_end)
    hit_end=True 表示新中心已越界（窗口放不下）。
    """
    half = window_size // 2
    u_new = np.float32(center_uv[0]) + np.float32(direction_uv[0]) * np.float32(stride)
    v_new = np.float32(center_uv[1]) + np.float32(direction_uv[1]) * np.float32(stride)

    # 中心的有效范围（保证正方形窗口完整在图像内）
    u_min, u_max = half, max(half, width - half)
    v_min, v_max = half, max(half, height - half)

    if (u_new < u_min) or (u_new >= u_max) or (v_new < v_min) or (v_new >= v_max):
        return ((center_uv[0], center_uv[1]), True)
    return ((u_new, v_new), False)

def quaternion_to_rot_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """(x, y, z, w) -> 3x3 旋转矩阵"""
    n = np.sqrt(x * x + y * y + z * z + w * w)
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