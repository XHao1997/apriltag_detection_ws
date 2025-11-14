Apriltag detection ROS2 node
=================================

This folder contains `apriltag_detection.py`, a small ROS2 node that subscribes to `/camera` (sensor_msgs/Image) and publishes detected AprilTag poses as `TF transform` on `/tag_point`.

Dependencies
------------
- ROS 2 (Foxy/Galactic/Humble or newer with Python rclpy)
- python packages: `pupil-apriltags`, `numpy`, `opencv-python`, `cv_bridge`

Install python deps in your venv or system python:

```bash
pip install pupil-apriltags numpy opencv-python 
# cv_bridge is usually provided by ROS packages; on Ubuntu install ros-<distro>-cv-bridge
```

Usage
-----
Make sure the script is executable (it already is in this repo):

```bash
ros2 run robot_vision apriltag_detection.py
```

You can also launch the script directly (if in a sourced ROS2 environment):

```bash
./apriltag_detection.py
```

Parameters
----------
The node declares parameters for camera intrinsics and tag size. Override them via ROS2 parameters when launching.

Notes
-----
This is a minimal node; for production you may want to:
- publish transforms to tf2
- publish one topic per tag id or an array message
- add camera_info subscription instead of hardcoding intrinsics
