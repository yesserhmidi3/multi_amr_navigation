import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    package_name = 'amr_map'
    pkg_path = get_package_share_directory(package_name)
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    urdf_path = os.path.join(pkg_path, 'urdf', 'amr_bot.urdf')
    rviz_config_file = os.path.join(pkg_path, 'config', 'robot_config.rviz')
    world_path = os.path.join(pkg_path, 'worlds', 'map_world.sdf')
    slam_params_file = os.path.join(pkg_path, 'config', 'slam_prams.yaml')

    with open(urdf_path, 'r') as f:
        urdf_raw = f.read()

    global_sim_time = SetParameter(name='use_sim_time', value=True)

    start_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/my_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/scan_front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/scan_rear@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/model/my_robot/tf', '/tf'),   
            ('/tf_static', '/tf_static')
        ],
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_raw,
            'use_sim_time': True,
        }],
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static')
        ],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', 'robot_description',
            '-x', '47.5',
            '-y', '12.3',
            '-z', '0.5',
            '-Y', '3.14159265'
        ],
        output='screen'
    )

    start_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'true'
        }.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': True
        }],
        output='screen'
    )

    teleop = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 'ros2', 'run', 'teleop_twist_keyboard',
            'teleop_twist_keyboard', '--ros-args', '-p', 'stamped:=false',
            '-r', 'cmd_vel:=cmd_vel'
        ],
        output='screen'
    )

    return LaunchDescription([
        global_sim_time,
        start_gazebo,
        ros_gz_bridge,
        robot_state_pub,
        spawn_robot,
        start_slam,
        rviz,
        teleop,
    ])