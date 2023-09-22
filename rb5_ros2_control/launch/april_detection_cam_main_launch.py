# example.launch.py
import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    #ld = LaunchDescription()

    # include_main_camera = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(
    #                 get_package_share_directory('rb5_ros2_vision'),
    #                 'launch/rb_camera_main_ocv_launch.py'))
    # )

    hw2_solution_node = Node(
        package='rb5_ros2_control',
        executable='hw2_solution.py',
        output='screen'
        )

    static_pub1 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '1.5', '--y', '0', '--z', '0', '--qx', '0.5', '--qy', '-0.5', '--qz', '0.5', '--qw', '-0.5', '--frame-id', 'map', '--child-frame-id', 'marker_0']
        )

    static_pub2 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '-0.5', '--y', '2', '--z', '0', '--qx', '-0.5', '--qy', '-0.5', '--qz', '0.5', '--qw', '0.5', '--frame-id', 'map', '--child-frame-id', 'marker_1']
        )


    return LaunchDescription([
        #include_main_camera,
        hw2_solution_node,
        static_pub1,
        static_pub2
    ])