#!/usr/bin/env python3
"""
Simple ROS 2 node:
- Subscribe to a camera image topic (sensor_msgs/Image)
- Detect AprilTags with pupil_apriltags
- Publish:
  - tag center + id + rotation as geometry_msgs/Transform on /tag_center_pixel
  - TF camera_color_optical_frame -> tag_link (or configured frame)
- Only when debug==True:
  - Print per-tag info
  - Publish annotated image (resized) on /tag_detections_img
"""

import os
import yaml
import numpy as np
import cv2

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
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
        self.declare_parameter('tag_size', 0.02975)  # [m]
        self.declare_parameter('tag_id', 0)
        self.declare_parameter(
            'camera_intrinsics_yaml',
            '/src/apriltag_detection/config/camera_parameter.yaml',
        )

        self.declare_parameter('use_rgb_3d_estimate', True)
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('debug', False)  # 是否调试输出和画图


        self.tag_id = self.get_parameter('tag_id').get_parameter_value().integer_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        tag_family = self.get_parameter('tag_family').get_parameter_value().string_value
        self.publish_frame = self.get_parameter('publish_frame').get_parameter_value().string_value
        self.tag_size = self.get_parameter('tag_size').get_parameter_value().double_value
        self.use_rgb_3d_estimate = self.get_parameter('use_rgb_3d_estimate').get_parameter_value().bool_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        camera_intrinsics_yaml = self.get_parameter(
            'camera_intrinsics_yaml'
        ).get_parameter_value().string_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value

        # ---------------- 相机内参 ----------------
        self.camera_params = self._load_camera_intrinsics(camera_intrinsics_yaml)

        # ---------------- 工具 & 检测器 ----------------
        self.bridge = CvBridge()
        self.detector = Detector(
                families="tag36h11",
                nthreads=1,
                quad_decimate=1.0,
                quad_sigma=0.8,
                refine_edges=1,
                decode_sharpening=0.25,
                debug=0
                )

        # ---------------- Publisher & Subscriber ----------------
        qos_image = qos_profile_sensor_data  # 预设的传感器数据 QoSProfile

        # debug 模式下我们才会真正往 /tag_detections_img 发图像
        self.detect_img_pub = self.create_publisher(Image, '/tag_detections_img', 20)
        # 用 Transform 携带 (px, py, tag_id, q)
        self.center_pub = self.create_publisher(Transform, '/tag_center_pixel', 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Image,
            image_topic,
            self.image_cb,
            qos_image,
        )

        self.get_logger().info(f'Subscribed to {image_topic}')
        self.get_logger().info(
            f'Publishing tag TF with parent "{self.camera_frame}" and child "{self.publish_frame}"'
        )
        self.get_logger().info(f'AprilTag detection node started. debug={self.debug}')

    # ---------------------------------------------------------
    # 加载相机内参 (fx, fy, cx, cy)，适配你的 YAML 结构
    # ---------------------------------------------------------
    def _load_camera_intrinsics(self, yaml_path):
        """
        从 YAML 读取相机内参 (fx, fy, cx, cy)
        适配如下结构：

        camera_intrinsics:
          coeffs: [...]
          fx: ...
          fy: ...
          ppx: ...   # cx
          ppy: ...   # cy
          width: ...
          height: ...
          depth_scale: ...
        """
        # 默认内参：直接用你给的那组作为 fallback
        default_params = (
            434.79052734375,      # fx
            426.10443115234375,   # fy
            434.458251953125,     # cx (ppx)
            241.777099609375,     # cy (ppy)
        )

        if not yaml_path or not os.path.exists(yaml_path):
            self.get_logger().warn(
                f'Camera intrinsics YAML not found: "{yaml_path}". '
                f'Using default parameters: {default_params}'
            )
            return default_params

        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)

            cam = data.get('camera_intrinsics', {})
            fx = float(cam['fx'])
            fy = float(cam['fy'])
            cx = float(cam['ppx'])   # principal point x
            cy = float(cam['ppy'])   # principal point y

            params = (fx, fy, cx, cy)
            self.get_logger().info(f'Loaded camera intrinsics from {yaml_path}: {params}')
            return params

        except Exception as e:
            self.get_logger().error(
                f'Failed to load intrinsics from {yaml_path}: {e}. '
                f'Using default parameters: {default_params}'
            )
            return default_params

    # ---------------------------------------------------------
    # 图像回调：检测 AprilTag + 发布结果
    # ---------------------------------------------------------
    def image_cb(self, msg: Image):
        # 1) ROS Image -> OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            if self.debug:
                self.get_logger().error(f'cv_bridge failed: {e}')
            return

        # 2) 灰度图
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY).astype(np.uint8)

        # 3) AprilTag 检测（在原始分辨率上做）
        tags = self.detector.detect(
            gray_image,
            camera_params=self.camera_params,
            estimate_tag_pose=self.use_rgb_3d_estimate,
            tag_size=self.tag_size,
        )

        if not tags:
            # 没检测到 tag，直接返回
            return

        # 4) 遍历每个检测结果
        for tag in tags:
            self._handle_single_tag(tag, msg)

        # 5) 仅在 debug 模式下画检测结果图像并发布
        if self.debug:
            annotated_full = draw_img_detections(cv_image.copy(), tags)
            try:
                # 压缩到 424x240（你配置的尺寸）
                target_width, target_height = 424, 240
                annotated_small = cv2.resize(
                    annotated_full,
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
        # center (像素坐标)
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
        pt = Transform()
        pt.translation.x = px                # 像素 x
        pt.translation.y = py                # 像素 y
        pt.translation.z = tag_id_float      # 用 z 携带 tag id

        pt.rotation.x = q[0]
        pt.rotation.y = q[1]
        pt.rotation.z = q[2]
        pt.rotation.w = q[3]

        self.center_pub.publish(pt)

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
                f'Detected tag ID: {tag_id} at pixel ({px:.1f}, {py:.1f})'
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
