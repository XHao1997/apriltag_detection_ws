#!/usr/bin/env python3
"""
Simple ROS2 node: subscribe to /camera (sensor_msgs/Image) and publish detected
AprilTag poses as geometry_msgs/PoseStamped on /tag_link.

Dependencies: rclpy, sensor_msgs, geometry_msgs, cv_bridge, numpy, pupil_apriltags, opencv-python

This is a minimal implementation intended to be placed under the package's
`scripts/` directory and made executable.
"""
import sys
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PointStamped, Transform
from cv_bridge import CvBridge
from pupil_apriltags import Detector
import cv2
from utils.realsense_helper import draw_img_detections, rotation_matrix_to_quaternion


class AprilTagNode(Node):
	def __init__(self):

		super().__init__('apriltag_detection')
		self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
		self.declare_parameter('tag_family', 'tag36h11')
		self.declare_parameter('publish_frame', 'tag_link')
		self.declare_parameter('tag_size', 0.162)  
		self.declare_parameter('camera_intrinsics_yaml', 'src/robot_vision/config/camera_parameter.yaml')
		image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
		tag_family = self.get_parameter('tag_family').get_parameter_value().string_value
		self.publish_frame = self.get_parameter('publish_frame').get_parameter_value().string_value
		self.tag_size = self.get_parameter('tag_size').get_parameter_value().double_value

		self.bridge = CvBridge()
		self.detector = Detector(families=tag_family,
			nthreads=1,
			quad_decimate=1.0,
			quad_sigma=0.0,
			refine_edges=1,
			decode_sharpening=0.25,
			debug=0)


		self.detect_img_pub = self.create_publisher(Image, '/tag_detections_img', 5)
		# publish each tag center in pixel coordinates: PointStamped (x=px, y=py, z=tag_id)
		self.center_pub = self.create_publisher(Transform, '/tag_center_pixel', 5)
		
		self.sub = self.create_subscription(Image, image_topic, self.image_cb, 5)
		self.get_logger().info(f'Subscribed to {image_topic}, publishing tag poses on /tag_link')
		self.get_logger().info("AprilTag detection running...")

	def image_cb(self, msg: Image):
		if self.detector is None:
			self.get_logger().error('Detector not initialized')
			return
		try:
			cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		except Exception as e:
			self.get_logger().error(f'cv_bridge failed: {e}')
			return
		gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY).astype(np.uint8)
		raw = self.detector.detect(gray_image, camera_params=(634.34, 607.415, 331.404, 239.84), 
							 		estimate_tag_pose=True, tag_size=self.tag_size)
		if raw is None:
			return
		if isinstance(raw, (list, tuple)):
			tags = list(raw)

		else:
			tags = [raw]

		if len(tags) == 0:
			# draw and publish annotated image even when no tags detected
			annotated = draw_img_detections(cv_image.copy(), tags)
			try:
				self.detect_img_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
			except Exception as e:
				self.get_logger().error(f'failed to publish detection image: {e}')
			return


		# publish per-tag center points (pixel coordinates)
		for tag in tags:
			if hasattr(tag, 'center') and tag.center is not None:
				try:
					px = float(tag.center[0])
					py = float(tag.center[1])
				except Exception:
					continue
				tag_q = rotation_matrix_to_quaternion(np.asarray(tag.pose_R))
				print(f'Detected rotation (quaternion): {tag_q}')
				pt = Transform()
				pt.translation.x = px
				pt.translation.y = py
				pt.translation.z = float(tag.tag_id)  # placeholder for tag ID
				pt.rotation.x = tag_q[0]
				pt.rotation.y = tag_q[1]
				pt.rotation.z = tag_q[2]
				pt.rotation.w = tag_q[3]
				# try multiple possible id attributes
				tag_id = getattr(tag, 'tag_id', None)
				if tag_id is None:
					tag_id = getattr(tag, 'id', None)
				if tag_id is None:
					tag_id = -1
				try:
					print(f'Detected tag ID: {tag_id}')
					pt.translation.z = float(tag_id)
				except Exception:
					pt.translation.z = float(-1)
				self.center_pub.publish(pt)

		# publish annotated image once per callback
		annotated = draw_img_detections(cv_image.copy(), tags)
		try:
			self.detect_img_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
		except Exception as e:
			self.get_logger().error(f'failed to publish detection image: {e}')
			

def main(args=None):
	rclpy.init(args=args)
	node = None
	try:
		node = AprilTagNode()
		rclpy.spin(node)
	except KeyboardInterrupt:
		# Gracefully handle Ctrl+C
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

