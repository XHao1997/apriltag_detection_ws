#!/usr/bin/env python3
"""
ROS 2 AprilTag node:

- Subscribe:
    /camera/camera/color/image_rect_raw (sensor_msgs/Image)
    /camera/camera/color/camera_info    (sensor_msgs/CameraInfo)

- Detect AprilTags with pupil_apriltags
- Use intrinsics & distortion from CameraInfo:
    - K (3x3)
    - D (distortion coefficients)
  Undistort image first, then detect on undistorted image.

- Publish:
    /tag_center_pixel (geometry_msgs/Transform)
        translation.x = pixel x
        translation.y = pixel y
        translation.z = tag_id (float)
        rotation      = orientation of tag (camera -> tag) as quaternion

    TF:
        camera_frame (param) -> publish_frame (param, default "tag_link")

    /tag_detections_img (sensor_msgs/Image, debug only)
        Undistorted + annotated image (resized to 424x240)
"""

import os
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Transform, TransformStamped
from cv_bridge import CvBridge
from pupil_apriltags import Detector
from tf2_ros import TransformBroadcaster

from utils.realsense_helper import (
    draw_img_detections,
    rotation_matrix_to_quaternion,
)


class AprilTagNode(Node):
    def __init__(self):
        super().__init__('apriltag_detection')

        # ---------------- 参数声明 ----------------
        self.declare_parameter('image_topic', '/camera/camera/color/image_rect_raw')
        self.declare_parameter('tag_family', 'tag36h11')
        self.declare_parameter('publish_frame', 'tag_link')
        self.declare_parameter('tag_size', 0.03)  # [m]
        self.declare_parameter('tag_id', 0)

        # use_rgb_3d_estimate: 是否基于相机内参进行 3D 位姿估计
        self.declare_parameter('use_rgb_3d_estimate', True)
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('debug', False)  # 是否调试输出和画图

        # ---------------- 读取参数 ----------------
        self.tag_id = self.get_parameter('tag_id').get_parameter_value().integer_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        tag_family = self.get_parameter('tag_family').get_parameter_value().string_value
        self.publish_frame = self.get_parameter('publish_frame').get_parameter_value().string_value
        self.tag_size = self.get_parameter('tag_size').get_parameter_value().double_value
        self.use_rgb_3d_estimate = self.get_parameter('use_rgb_3d_estimate').get_parameter_value().bool_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value

        # ---------------- 相机内参 / 畸变，从 CameraInfo 获取 ----------------
        self.K = None                 # 3x3 内参矩阵
        self.dist_coeffs = None       # 畸变系数向量 D
        self.camera_params = None     # (fx, fy, cx, cy) 给 pupil_apriltags 用
        self.cam_info_received = False

        # ---------------- 工具 & 检测器 ----------------
        self.bridge = CvBridge()
        self.detector = Detector(
            families=tag_family,
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=0,
            decode_sharpening=0.25,
            debug=0
        )

        # ---------------- Publisher & Subscriber ----------------
        qos_image = qos_profile_sensor_data

        # Debug 图像（仅 debug 时真正使用）
        self.detect_img_pub = self.create_publisher(Image, '/tag_detections_img', 20)

        # 用 Transform 携带 (px, py, tag_id, q)
        self.center_pub = self.create_publisher(Transform, '/tag_center_pixel', 20)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # 图像订阅
        self.sub = self.create_subscription(
            Image,
            image_topic,
            self.image_cb,
            qos_image,
        )

        # CameraInfo 订阅（用来获取 K 和 D）
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_cb,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f'Subscribed to image: {image_topic}')
        self.get_logger().info(f'Subscribed to camera_info: /camera/camera/color/camera_info')
        self.get_logger().info(
            f'Publishing tag TF with parent "{self.camera_frame}" and child "{self.publish_frame}"'
        )
        self.get_logger().info(f'AprilTag detection node started. debug={self.debug}')

    # ---------------------------------------------------------
    # CameraInfo 回调：初始化 K、畸变系数、camera_params
    # ---------------------------------------------------------
    def camera_info_cb(self, msg: CameraInfo):
        if self.cam_info_received:
            return

        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64).reshape(-1)

        fx = float(K[0, 0])
        fy = float(K[1, 1])
        cx = float(K[0, 2])
        cy = float(K[1, 2])

        self.K = K
        self.dist_coeffs = D
        self.camera_params = (fx, fy, cx, cy)
        self.cam_info_received = True

        self.get_logger().info(
            f'CameraInfo received: fx={fx:.3f}, fy={fy:.3f}, '
            f'cx={cx:.3f}, cy={cy:.3f}, D={D.tolist()}'
        )

        # 如果你希望 camera_frame 自动跟随 camera_info，可以在这里启用：
        # self.camera_frame = msg.header.frame_id
        # self.get_logger().info(f'camera_frame set to "{self.camera_frame}" from CameraInfo')

    # ---------------------------------------------------------
    # 图像回调：去畸变 + AprilTag 检测 + 发布
    # ---------------------------------------------------------
    def image_cb(self, msg: Image):
        # CameraInfo 还没来，不做检测
        if not self.cam_info_received or self.K is None or self.camera_params is None:
            if self.debug:
                self.get_logger().debug('Waiting for CameraInfo...')
            return

        # 1) ROS Image -> OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            if self.debug:
                self.get_logger().error(f'cv_bridge failed: {e}')
            return

        # 2) 灰度图
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY).astype(np.uint8)

        # 2.5) 去畸变（如果 D 有非零内容）
        if self.dist_coeffs is not None and self.dist_coeffs.size > 0:
            # 去畸变灰度图用于检测
            gray_for_detect = cv2.undistort(gray_image, self.K, self.dist_coeffs)
            # 可选：去畸变彩色图用于可视化
            color_for_vis = cv2.undistort(cv_image, self.K, self.dist_coeffs)
        else:
            gray_for_detect = gray_image
            color_for_vis = cv_image

        # 3) AprilTag 检测（在去畸变图像上）
        tags = self.detector.detect(
            gray_for_detect,
            camera_params=self.camera_params,          # (fx, fy, cx, cy) 对应 undistort 后图像
            estimate_tag_pose=self.use_rgb_3d_estimate,
            tag_size=self.tag_size,
        )

        if not tags:
            return

        # 4) 按检测结果处理（发布 /tag_center_pixel & TF）
        for tag in tags:
            self._handle_single_tag(tag, msg)

        # 5) debug 模式下画检测结果并发布 /tag_detections_img
        if self.debug:
            annotated = draw_img_detections(color_for_vis.copy(), tags)
            try:
                target_width, target_height = 424, 240
                annotated_small = cv2.resize(
                    annotated,
                    (target_width, target_height),
                    interpolation=cv2.INTER_AREA
                )

                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_small, encoding='bgr8')
                annotated_msg.header.stamp = msg.header.stamp
                annotated_msg.header.frame_id = msg.header.frame_id
                self.detect_img_pub.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f'failed to publish detection image: {e}')

    # ---------------------------------------------------------
    # 处理单个 Tag：发布 center + TF
    # ---------------------------------------------------------
    def _handle_single_tag(self, tag, image_msg: Image):
        # center (像素坐标，基于去畸变后的图像)
        if not (hasattr(tag, 'center') and tag.center is not None):
            return

        try:
            px = float(tag.center[0])
            py = float(tag.center[1])
        except Exception:
            return

        # tag id
        tag_id = getattr(tag, 'tag_id', None)
        if tag_id is None:
            tag_id = getattr(tag, 'id', -1)
        try:
            tag_id_float = float(tag_id)
        except Exception:
            tag_id_float = -1.0

        # rotation (camera -> tag)
        if hasattr(tag, 'pose_R'):
            q = rotation_matrix_to_quaternion(np.asarray(tag.pose_R))
        else:
            q = (0.0, 0.0, 0.0, 1.0)

        # ---- 发布到 /tag_center_pixel （用 Transform 携带信息）----
        tf_msg = Transform()
        tf_msg.translation.x = px               # 像素 x
        tf_msg.translation.y = py               # 像素 y
        tf_msg.translation.z = tag_id_float     # tag id

        tf_msg.rotation.x = q[0]
        tf_msg.rotation.y = q[1]
        tf_msg.rotation.z = q[2]
        tf_msg.rotation.w = q[3]

        self.center_pub.publish(tf_msg)

        # ---- 发布 TF camera -> tag_link ----
        if self.use_rgb_3d_estimate and hasattr(tag, 'pose_t') and hasattr(tag, 'pose_R'):
            t = TransformStamped()
            t.header.stamp = image_msg.header.stamp
            t.header.frame_id = self.camera_frame    # e.g. camera_color_optical_frame
            t.child_frame_id = self.publish_frame    # e.g. tag_link

            # pose_t: (tx, ty, tz) in meters (camera -> tag)
            tx, ty, tz = map(float, np.asarray(tag.pose_t).reshape(-1))
            t.transform.translation.x = tx
            t.transform.translation.y = ty
            t.transform.translation.z = tz

            # pose_R -> quaternion
            qx, qy, qz, qw = q
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(t)

        if self.debug:
            self.get_logger().info(
                f'Detected tag ID: {tag_id} at undistorted pixel ({px:.1f}, {py:.1f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AprilTagNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
