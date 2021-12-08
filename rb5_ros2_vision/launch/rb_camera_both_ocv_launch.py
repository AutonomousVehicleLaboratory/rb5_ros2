# example.launch.py

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # include another launch file
    launch_include_main = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rb5_ros2_vision'),
                'launch/rb_camera_main_ocv_launch.py'))
    )

    launch_include_side = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rb5_ros2_vision'),
                'launch/rb_camera_side_ocv_launch.py'))
    )

    return LaunchDescription([
        launch_include_main,
        launch_include_side
    ])
