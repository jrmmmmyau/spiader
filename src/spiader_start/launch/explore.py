from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
		IncludeLaunchDescription(
				PythonLaunchDescriptionSource([
				os.path.join(get_package_share_directory('spiader_start'), 'launch', 'teleop_slam.py')
				])
		),
		IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
            ]),
            launch_arguments={
                'params_file': os.path.join(
                    get_package_share_directory('navigation'), 'yaml', 'nav2_params.yaml'
                ),
                'use_docking': 'False',
                'log_level': 'warn'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('explore_lite'), 'launch', 'explore.launch.py')
            ]),
            launch_arguments={
                'params_file': os.path.join(
                    get_package_share_directory('navigation'), 'yaml', 'explore.yaml'
                ),
                'log_level': 'warn'
            }.items()
        ),

        Node(
            package='navigation',
            executable='ending_search'
        )


])
