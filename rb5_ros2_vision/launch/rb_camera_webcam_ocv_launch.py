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
    camera_id = DeclareLaunchArgument(
        "camera_id", default_value=TextSubstitution(text="2")
    )
    frame_rate = DeclareLaunchArgument(
        "frame_rate", default_value=TextSubstitution(text="30")
    )
    width = DeclareLaunchArgument(
        "width", default_value=TextSubstitution(text="640")
    )
    height = DeclareLaunchArgument(
        "height", default_value=TextSubstitution(text="480")
    )
    input_format = DeclareLaunchArgument(
        "input_format", default_value=TextSubstitution(text="YUY2")
    )
    output_format = DeclareLaunchArgument(
        "output_format", default_value=TextSubstitution(text="RGB")
    )
    topic_name = DeclareLaunchArgument(
        "topic_name", default_value=TextSubstitution(text="camera_2")
    )
    image_compress = DeclareLaunchArgument(
        "image_compress", default_value=TextSubstitution(text="false")
    )

    # start another turtlesim_node in the turtlesim2 namespace
    # and use args to set parameters
    rb_camera_main_ocv_node = Node(
            package='rb5_ros2_vision',
            node_executable='rb_camera_ocv_node',
            name='rb_camera_webcam_ocv',
            parameters=[{
                "camera_id": LaunchConfiguration('camera_id'),
                "frame_rate": LaunchConfiguration('frame_rate'),
                "width": LaunchConfiguration('width'),
                "height": LaunchConfiguration('height'),
                "input_format": LaunchConfiguration('input_format'),
                "output_format": LaunchConfiguration('output_format'),
                "topic_name": LaunchConfiguration('topic_name'),
                "image_compress": LaunchConfiguration('image_compress'),
            }]
        )

    return LaunchDescription([
        camera_id,
        frame_rate,
        width,
        height,
        input_format,
        output_format,
        topic_name,
        image_compress,
        rb_camera_main_ocv_node
    ])