#!/usr/bin/env python3
import os
import sys
import tty
import cv2
import select
import termios
import threading
from pathlib import Path
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge


class ImageRecorder(Node):
    def __init__(self):
        super().__init__('image_recorder')

        # ---- Params ----
        self.declare_parameter('image_topic', '/camera/camera/color/image_rect_raw')
        self.declare_parameter('data_dir', 'data')
        self.declare_parameter('detection_topic', '/tree_base_detection')

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        data_dir = self.get_parameter('data_dir').get_parameter_value().string_value
        detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value

        # ---- Paths ----
        self.data_dir = Path(data_dir).resolve()
        self.data_dir.mkdir(parents=True, exist_ok=True)

        # ---- Bridge & flags ----
        self.bridge = CvBridge()
        self._save_event = threading.Event()   # 一次性保存请求（手动/自动/来自检测）
        self._auto_record = False             # 键盘触发的自动录制

        # ---- QoS: sensor data style (best-effort, keep last) ----
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---- Image Sub ----
        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, qos
        )

        # ---- Detection flag Sub ----
        self.detection_sub = self.create_subscription(
            Int32, detection_topic, self.detection_callback, 10
        )

        # ---- 定时器：每 0.5 秒触发一次 ----
        # 如果自动模式开启（_auto_record=True），定时器回调会设置 _save_event
        self.timer = self.create_timer(0.5, self._auto_record_timer_cb)

        # ---- Keyboard thread (only if stdin is a TTY) ----
        if sys.stdin.isatty():
            self._kb_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
            self._kb_thread.start()
            self.get_logger().info(
                'Image recorder started.\n'
                '  [s] save one image\n'
                '  [a] toggle auto record every 0.5s\n'
                '  [q] quit\n'
                'Additionally: any time /tree_base_detection == 0, one image will be saved.'
            )
        else:
            self.get_logger().warn(
                'STDIN is not a TTY (probably launched from a non-interactive shell). '
                'Keyboard controls disabled. You can still save via detection topic or ROS params/actions.'
            )

    # ---------- Detection callback ----------
    def detection_callback(self, msg: Int32):
        # 如果检测结果为 0（未检测到树基），请求保存一张图
        if msg.data == 0:
            self._save_event.set()
            self.get_logger().info('Detection flag = 0, save requested.')

    # ---------- Image callback ----------
    def image_callback(self, msg: Image):
        try:
            # Convert ROS Image -> OpenCV BGR
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Only save when requested (and then clear the request)
            if self._save_event.is_set():
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
                filename = f'image_{timestamp}.jpg'
                filepath = self.data_dir / filename

                ok = cv2.imwrite(str(filepath), cv_image)
                if ok:
                    self.get_logger().info(f'Saved image: {filename}')
                else:
                    self.get_logger().error(f'Failed to write image file: {filepath}')

                # 这次保存完就清掉事件
                self._save_event.clear()

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    # ---------- Auto record timer callback ----------
    def _auto_record_timer_cb(self):
        # 如果已经开启自动录制，每 0.5 秒请求保存一张
        if self._auto_record:
            self._save_event.set()

    # ---------- Keyboard handling (Unix) ----------
    def _keyboard_listener(self):
        # Guard against termios errors even with isatty()
        try:
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
        except Exception as e:
            self.get_logger().warn(f'Keyboard disabled (termios init failed): {e}')
            return

        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                r, _, _ = select.select([sys.stdin], [], [], 0.1)
                if r:
                    key = sys.stdin.read(1)
                    if key in ('s', 'S'):
                        # 单次保存
                        self._save_event.set()
                        self.get_logger().info('Save requested (single).')
                    elif key in ('a', 'A'):
                        # 自动录制开关
                        self._auto_record = not self._auto_record
                        state = 'ON' if self._auto_record else 'OFF'
                        self.get_logger().info(f'Auto record toggled: {state} (interval = 0.5s)')
                    elif key in ('q', 'Q'):
                        self.get_logger().info('Quit requested.')
                        # Trigger shutdown from this thread
                        rclpy.shutdown()
                        break
        except Exception as e:
            self.get_logger().warn(f'Keyboard listener stopped: {e}')
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ImageRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
