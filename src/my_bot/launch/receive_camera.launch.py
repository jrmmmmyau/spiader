import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    launch_camera = Node(
        package='image_view',
        executable='image_view',
        remappings=[
            ('image', '/image_raw')
        ],
        parameters=[
            {'image_transport': 'compressed'}
        ]
    )


    # Launch!
    return LaunchDescription([
        launch_camera
    ])
