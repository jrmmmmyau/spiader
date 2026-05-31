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
				os.path.join(get_package_share_directory('my_bot'), 'launch', 'launch_robot.launch.py')
				])
		),
		IncludeLaunchDescription(
				PythonLaunchDescriptionSource([
				os.path.join(get_package_share_directory('sensor'), 'launch', 'launch_camera.py')
				])
		),   
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_c1_launch.py')
                ])
        ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'),
                ]),

                launch_arguments={'use_sim_time': 'false'}.items()
        )


])
