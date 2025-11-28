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
            
                'depth_module.profile': '848x480x15',
                'rgb_camera.profile': '848x480x15',
                'rgbd_mode.enable': 'True',
                'enable_color': 'True',
                'clip_distance': '1.5', 
                'pointcloud.enable':'True',
                'align_depth.enable':'True',
                'initial_reset': 'True'
            }.items()
    )
    
    # Launch them all!
    return LaunchDescription([
        rs_camera,
    ])