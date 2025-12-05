import os
from ament_index_python.packages import get_package_share_directory  # <-- correct import

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    realsense_package_name = 'realsense2_camera'
    depth2cloud2_package_name = 'depthimage_to_pointcloud2'


    # RealSense camera (aligned depth + profiles)
    rs_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(realsense_package_name),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
                'depth_module.profile': '484x480x10',
                'enable_color': 'True',
                'clip_distance': '1.5', 
                'enable_rgbd.enable':'True',
                'align_depth.enable':'True',
                'initial_reset': 'True'
        }.items()
    )

    # Parameter file for camera intrinsics (used by apriltag_tf.py)
    project_name = 'soil_task_vision'  # project name

    # Prefer an absolute path resolved from the package share directory so
    # the node can open the file regardless of the current working directory.
    camera_intrinsics = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory(project_name),
            'config',
            'camera_parameter.yaml'
        ),
        description='Full path to the parameter file to load'
    )

    # AprilTag detector subscribes to the color image
    apriltag_node = Node(
        package=project_name,
        executable='apriltag_detection.py',
        name='apriltag_detection',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'image_topic': '/camera/camera/color/image_rect_raw',            # fixed topic
        }],
    )

    # TF node uses tag center + aligned depth
    apriltag_tf_node = Node(
        package=project_name,
        executable='apriltag_tf.py',
        name='apriltag_tf',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'tag_center_topic': '/tag_center_pixel',
            'depth_topic': '/camera/camera/depth/image_rect_raw',  # fixed topic
            'intrinsics_yaml': LaunchConfiguration('params_file'),      # <- use the launch arg
            'camera_frame': 'camera_color_optical_frame',
        }],
    )
    # RViz2 visualization
    rviz_config = os.path.join(
        get_package_share_directory(project_name),
        'config',
        'default.rviz'
    )
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        emulate_tty=True,
        arguments=['-d', rviz_config],
    )


    depth2cloud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory(depth2cloud2_package_name),
                        'launch',
                        'depthimage_to_pointcloud2.launch.py'
                    )
                )
    )

    return LaunchDescription([
        camera_intrinsics,   
        rs_camera,
        # depth2cloud_launch,
        # apriltag_tf_node,
        # rviz2
    ])
