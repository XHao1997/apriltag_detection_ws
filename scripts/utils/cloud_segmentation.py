# apriltag_detection/scripts/utils/pc_utils.py
import numpy as np
from typing import List, Tuple, Optional
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformException
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
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


# ---------- TF 转换到目标坐标系 ----------
def transform_cloud_to_frame(
    tf_buffer: Buffer,
    cloud_msg: PointCloud2,
    target_frame: Optional[str],
    logger=None
) -> PointCloud2:
    if not target_frame:
        return cloud_msg
    try:
        tf = tf_buffer.lookup_transform(
            target_frame,
            cloud_msg.header.frame_id,
            rclpy.time.Time()
        )
        return do_transform_cloud(cloud_msg, tf)
    except TransformException as ex:
        if logger:
            logger.warn(
                f"TF transform {cloud_msg.header.frame_id} -> {target_frame} failed: {ex}. Use original frame."
            )
        return cloud_msg


# ---------- 取窗口内点（正方形） ----------
def extract_square_window_points(
    points_2d: np.ndarray,
    center_uv: list[float],
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
def mean_normal_over_frames(windows: List[np.ndarray]) -> Tuple[np.ndarray, np.ndarray, float]:
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
    stability = float(np.linalg.norm(normals.std(axis=0)))
    return mean_normal, mean_centroid, stability


# ---------- 箭头两端点 ----------
def arrow_points_from_normal(centroid: np.ndarray, normal: np.ndarray, length: float = 0.2) -> Tuple[Point, Point]:
    start = centroid.astype(float)
    end   = (centroid + normal * length).astype(float)
    p0 = Point(x=float(start[0]), y=float(start[1]), z=float(start[2]))
    p1 = Point(x=float(end[0]),   y=float(end[1]),   z=float(end[2]))
    return p0, p1


# ---------- 识别点 → 图像底部中点 的扫描方向 ----------
def compute_scan_direction(
    recog_uv: Tuple[float, float],
    width: int, height: int
) -> np.ndarray:
    bottom_mid = np.array([width / 2.0, height - 1.0], dtype=float)
    recog = np.array(recog_uv, dtype=float)
    v = bottom_mid - recog
    n = np.linalg.norm(v)
    if n < 1e-6:
        # 识别点刚好在底中，默认向下
        return np.array([0.0, 1.0], dtype=float)
        print("recg locate at mid")
    return v / n


# ---------- 沿方向推进窗口中心 ----------
def step_along_direction(
    center_uv: list[float],
    direction_uv: list[float],
    stride: float,
    window_size: int,
    width: int, height: int
) -> Tuple[Tuple[float, float], bool]:
    """
    沿给定方向把窗口中心推进 stride 像素。
    返回：(new_center_uv, hit_end)
    hit_end=True 表示新中心已越界（窗口放不下）。
    """
    half = window_size // 2
    u_new = float(center_uv[0]) + float(direction_uv[0]) * float(stride)
    v_new = float(center_uv[1]) + float(direction_uv[1]) * float(stride)

    # 中心的有效范围（保证正方形窗口完整在图像内）
    u_min, u_max = half, max(half, width - half)
    v_min, v_max = half, max(half, height - half)

    if (u_new < u_min) or (u_new >= u_max) or (v_new < v_min) or (v_new >= v_max):
        return (center_uv, True)
    return ((u_new, v_new), False)
