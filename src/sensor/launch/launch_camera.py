from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([ 
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            parameters=[os.path.join(get_package_share_directory('sensor'), 'config', 'camera.yaml')
]
            
        )
    ])
