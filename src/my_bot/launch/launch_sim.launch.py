import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction

from launch_ros.actions import Node



def generate_launch_description():
    # This tells Gazebo to use the CPU for graphics, fixing the invisible rays
    env_var_1 = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')
    env_var_2 = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
     os.path.join(get_package_share_directory('my_bot'), 'worlds'))

    # robot_state_publisher 

    package_name='my_bot' 

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Gazebo launch file
    my_custom_world = '/home/jrm/dev_ws/src/my_bot/worlds/obstacles.sdf'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 1 {my_custom_world}'}.items(), #the number here is the level of warning
    )
    # gazebo = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
    #             )]), launch_arguments={'gz_args': '-r empty.sdf'}.items()
                    
    # )

    # spawner node 
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot',
                                   '-z', '0.1'],
                        output='screen')

    # for compressing image
    republish_rgb = Node(
        package='image_transport',
        executable='republish',
        arguments=['raw', 'in:=/camera/image_raw', 'compressed', 'out:=/camera/image_raw'],
        remappings= #ros is ignoring this, so the type is still out
        [
            ('in', '/camera/image_raw'),
            ('out', '/camera/image/raw')
        ],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        remappings=[
            ('/model/robot/tf', '/tf')
        ],
        output='screen'
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )


  
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_controller", 
            "--param-file", os.path.join(get_package_share_directory('my_bot'), 'config', 'my_controllers.yaml')
        ],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        prefix='xterm -e', 
        remappings=[
            # ('/cmd_vel', '/diff_controller/cmd_vel')
        ]
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
        gazebo,
        spawn_entity,
        bridge,
        republish_rgb,
        # diff_drive_spawner,
        # joint_broad_spawner,
        teleop_node,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        # joint_state_publisher,
        twist_stamper
    ])