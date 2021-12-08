# example.launch.py

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    # args that can be set from the command line or a default will be used
    use_rb_cam = DeclareLaunchArgument(
        "use_rb_cam", default_value=TextSubstitution(text="true")
    )
    camera_id = DeclareLaunchArgument(
        "camera_id", default_value=TextSubstitution(text="0")
    )
    frame_rate = DeclareLaunchArgument(
        "frame_rate", default_value=TextSubstitution(text="30")
    )
    width = DeclareLaunchArgument(
        "width", default_value=TextSubstitution(text="1920")
    )
    height = DeclareLaunchArgument(
        "height", default_value=TextSubstitution(text="1080")
    )
    input_format = DeclareLaunchArgument(
        "input_format", default_value=TextSubstitution(text="NV12")
    )
    output_format = DeclareLaunchArgument(
        "output_format", default_value=TextSubstitution(text="RGB")
    )
    topic_name = DeclareLaunchArgument(
        "topic_name", default_value=['camera_', LaunchConfiguration('camera_id')]
    )
    image_compress = DeclareLaunchArgument(
        "image_compress", default_value=TextSubstitution(text="false")
    )
    image_rectify = DeclareLaunchArgument(
        "image_rectify", default_value=TextSubstitution(text="false")
    )
    camera_parameter_path = DeclareLaunchArgument(
        "camera_parameter_path", default_value=TextSubstitution(text="/root/dev/ros2ws/src/rb5_ros2/rb5_ros2_vision/config/camera_main.yaml")
    )

    # start another turtlesim_node in the turtlesim2 namespace
    # and use args to set parameters
    rb_camera_main_ocv_node = Node(
            package='rb5_ros2_vision',
            node_executable='rb_camera_ocv_node',
            name='rb_camera_main_ocv',
            output='screen',
            parameters=[{
                "use_rb_cam": LaunchConfiguration('use_rb_cam'),
                "camera_id": LaunchConfiguration('camera_id'),
                "frame_rate": LaunchConfiguration('frame_rate'),
                "width": LaunchConfiguration('width'),
                "height": LaunchConfiguration('height'),
                "input_format": LaunchConfiguration('input_format'),
                "output_format": LaunchConfiguration('output_format'),
                "topic_name": LaunchConfiguration('topic_name'),
                "image_compress": LaunchConfiguration('image_compress'),
                "image_rectify": LaunchConfiguration('image_rectify'),
                "camera_parameter_path": LaunchConfiguration("camera_parameter_path")
            }]
        )

    return LaunchDescription([
        use_rb_cam,
        camera_id,
        frame_rate,
        width,
        height,
        input_format,
        output_format,
        topic_name,
        image_compress,
        image_rectify,
        camera_parameter_path,
        rb_camera_main_ocv_node
    ])
