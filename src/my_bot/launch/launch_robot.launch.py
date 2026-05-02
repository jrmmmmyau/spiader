import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction

from launch_ros.actions import Node



def generate_launch_description():


    package_name='my_bot' 

    rsp = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory(package_name),'launch','rsp.launch.py'
    )]), launch_arguments={'use_sim_time': 'false'}.items()
)
  
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_controller", 
            "--param-file", os.path.join(get_package_share_directory('my_bot'), 'config', 'my_controllers_real.yaml')
        ],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    #this is for keyboard
    # teleop_node = Node(
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     name='teleop',
    #     prefix='xterm -e', 
    #     remappings=[
    #         # ('/cmd_vel', '/diff_controller/cmd_vel')
    #     ]
    # )


    #this is for controller
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen'
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop',
        output='screen',
        parameters=[{
            'axis_linear.x': 1,
            'axis_angular.yaw': 0,
            'scale_linear.x': 1.0,
            'scale_angular.yaw': 1.5,
            'require_enable_button': False,
        }]
    )
    controller_manager = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[
        os.path.join(get_package_share_directory('my_bot'), 'config', 'my_controllers_real.yaml')
    ],
    output='screen'
)


    delayed_diff_drive_spawner = TimerAction(
        period=5.0,
        actions=[diff_drive_spawner]
    )

    delayed_joint_broad_spawner = TimerAction(
        period=5.0,
        actions=[joint_broad_spawner]
    )


    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        remappings=[
            ('/cmd_vel_in', '/cmd_vel'),              # From teleop
            ('/cmd_vel_out', '/diff_controller/cmd_vel') # To controller
        ]
    )


    # Launch them all!
    return LaunchDescription([
        rsp,
        controller_manager,
        teleop_node,
        joy_node,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        twist_stamper
    ])
