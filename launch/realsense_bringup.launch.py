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
        )]), launch_arguments={
            
                'depth_module.depth_profile': '640x480x60',
                'depth_module.infra_profile': '640x480x60', 
                'rgb_camera.color_profile': '640x480x60',
                'enable_depth': 'True',
                'enable_color': 'True',
                'clip_distance': '1.5', 
                'pointcloud.enable':'False',
                'align_depth.enable':'True',
                'initial_reset': 'True'
            }.items()
    )
    
    # Launch them all!
    return LaunchDescription([
        rs_camera,
    ])