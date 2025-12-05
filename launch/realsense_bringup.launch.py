import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    realsense_package_name = 'realsense2_camera'
    
    # Launch Realsense camera launch file with aligned depth images publisher
    rs_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(realsense_package_name), 'launch', 'rs_launch.py'
        )]),
        launch_arguments={
                'initial_reset': 'True',
                'depth_module.depth_profile': '848x480x10',
                "rgb_camera.profile":"848x480x10",
                "accelerate_gpu_with_glsl.enable" : 'True',
                'align_depth.enable': 'True',

                # 'pointcloud.enable': 'True',
                'pointcloud.ordered_pc': 'True',
            }.items()
    )
    
    # Launch them all!
    return LaunchDescription([
        rs_camera,
    ])
