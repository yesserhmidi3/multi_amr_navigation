import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap, SetParameter
from launch.actions import ExecuteProcess, TimerAction, GroupAction

def get_namespaced_urdf(urdf_str, ns):
    s = urdf_str
    s = s.replace('<frame_id>odom</frame_id>', f'<frame_id>{ns}/odom</frame_id>')
    s = s.replace('<child_frame_id>base_footprint</child_frame_id>', f'<child_frame_id>{ns}/base_footprint</child_frame_id>')
    s = s.replace('<gz_frame_id>laser_front</gz_frame_id>', f'<gz_frame_id>{ns}/laser_front</gz_frame_id>')
    s = s.replace('<gz_frame_id>laser_rear</gz_frame_id>',  f'<gz_frame_id>{ns}/laser_rear</gz_frame_id>')
    s = s.replace('<gz_frame_id>imu_link</gz_frame_id>',    f'<gz_frame_id>{ns}/imu_link</gz_frame_id>')
    s = s.replace('<topic>cmd_vel</topic>',     f'<topic>/model/{ns}/cmd_vel</topic>')
    s = s.replace('<odom_topic>odom</odom_topic>', f'<odom_topic>/model/{ns}/odom</odom_topic>')
    s = s.replace('<topic>scan_front</topic>',  f'<topic>/model/{ns}/scan_front</topic>')
    s = s.replace('<topic>scan_rear</topic>',   f'<topic>/model/{ns}/scan_rear</topic>')
    return s

def generate_launch_description():
    package_name = 'amr_map_mult'
    pkg_path = get_package_share_directory(package_name)

    map_yaml = os.path.join(pkg_path, 'maps', 'map.yaml')
    urdf_path = os.path.join(pkg_path, 'urdf', 'amr_bot.urdf')
    world_path = os.path.join(pkg_path, 'worlds', 'map_world.sdf')
    params_r1 = os.path.join(pkg_path, 'config', 'nav2_params_robot_1.yaml')
    params_r2 = os.path.join(pkg_path, 'config', 'nav2_params_robot_2.yaml')
    rviz_config_file = os.path.join(pkg_path, 'config', 'robot_config.rviz')

    with open(urdf_path, 'r') as f:
        urdf_raw = f.read()

    global_sim_time = SetParameter(name='use_sim_time', value=True)
    start_gazebo = ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', world_path], output='screen')

    global_clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='global_clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[{'use_sim_time': True, 'yaml_filename': map_yaml}],
        remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
    )

    lifecycle_map = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map', output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True, 'bond_timeout': 0.0, 'node_names': ['map_server']}],
    )

    def make_robot(ns, params_file, sx, sy, syaw):
        urdf_str = get_namespaced_urdf(urdf_raw, ns)

        state_pub = Node(
            package='robot_state_publisher', executable='robot_state_publisher', namespace=ns, output='screen',
            parameters=[{'robot_description': urdf_str, 'use_sim_time': True, 'frame_prefix': f'{ns}/'}],
            remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
        )

        spawn = Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=['-name', ns, '-string', urdf_str, '-x', sx, '-y', sy, '-z', '0.5', '-Y', syaw],
        )

        bridge = Node(
            package='ros_gz_bridge', executable='parameter_bridge', namespace=ns, output='screen',
            arguments=[
                f'/model/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                f'/model/{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                f'/model/{ns}/scan_front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'/model/{ns}/scan_rear@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'/model/{ns}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            remappings=[
                (f'/model/{ns}/cmd_vel', f'/{ns}/cmd_vel'),
                (f'/model/{ns}/odom', f'/{ns}/odom'),
                (f'/model/{ns}/scan_front', f'/{ns}/scan_front'),
                (f'/model/{ns}/scan_rear', f'/{ns}/scan_rear'),
                (f'/model/{ns}/tf', '/tf'),
            ],
        )

        #ekf node to publish odom -> base_footprint transform
        ekf = Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_node', namespace=ns, output='screen',
            parameters=[{
                'use_sim_time': True,
                'odom_frame':      f'{ns}/odom',
                'base_link_frame': f'{ns}/base_footprint',
                'world_frame':     f'{ns}/odom',
                'odom0': f'/{ns}/odom',
                'odom0_config': [True, True, False,
                                 False, False, True,
                                 True, True, False,
                                 False, False, True,
                                 False, False, False],
                'publish_tf': True,
            }],
            remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
        )

        scan_merger = Node(
            package='amr_map_mult', executable='scan_merger', namespace=ns, output='screen',
            parameters=[{'use_sim_time': True, 'robot_name': ns}],
        )

        obstacle_pub = Node(
            package='amr_map_mult', executable='robot_obstacle_publisher', namespace=ns, output='screen',
            parameters=[{'use_sim_time': True, 'robot_name': ns}],
        )

        common = [params_file, {'use_sim_time': True}]

        amcl = Node(package='nav2_amcl', executable='amcl', name='amcl', namespace=ns, output='screen', parameters=common)
        controller = Node(package='nav2_controller', executable='controller_server', name='controller_server',
                           namespace=ns, output='screen', parameters=common, remappings=[('cmd_vel', 'cmd_vel_nav')])
        smoother = Node(package='nav2_smoother', executable='smoother_server', name='smoother_server', namespace=ns, output='screen', parameters=common)
        planner = Node(package='nav2_planner', executable='planner_server', name='planner_server', namespace=ns, output='screen', parameters=common)
        behavior = Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', namespace=ns, output='screen', parameters=common)
        bt_nav = Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', namespace=ns, output='screen', parameters=common)
        waypoint = Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower', namespace=ns, output='screen', parameters=common)
        vel_smoother = Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
                             namespace=ns, output='screen', parameters=common, remappings=[('cmd_vel', 'cmd_vel_nav')])
        collision = Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor', namespace=ns, output='screen', parameters=common)

        lifecycle_nav = Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', namespace=ns, output='screen',
            parameters=[{'use_sim_time': True, 'autostart': True, 'bond_timeout': 0.0,
                         'node_names': ['amcl', 'controller_server', 'smoother_server', 'planner_server',
                                        'behavior_server', 'bt_navigator', 'waypoint_follower',
                                        'velocity_smoother', 'collision_monitor']}],
        )

        nav2_group = GroupAction(actions=[
            SetRemap(src='tf', dst='/tf'), SetRemap(src='tf_static', dst='/tf_static'), SetRemap(src='map', dst='/map'),
            amcl, controller, smoother, planner, behavior, bt_nav, waypoint, vel_smoother, collision, lifecycle_nav,
        ])

        return state_pub, spawn, bridge, ekf, scan_merger, obstacle_pub, nav2_group

    (r1_sp, r1_spawn, r1_br, r1_ekf, r1_sm, r1_obs, r1_nav2) = make_robot('robot_1', params_r1, '47.5', '12.3', '3.14159265')
    (r2_sp, r2_spawn, r2_br, r2_ekf, r2_sm, r2_obs, r2_nav2) = make_robot('robot_2', params_r2, '47.5', '15.0475', '3.14159265')

    r1_delayed = TimerAction(period=3.0, actions=[r1_nav2])
    r2_delayed = TimerAction(period=6.0, actions=[r2_spawn, r2_ekf, r2_sm, r2_obs, r2_nav2])

    return LaunchDescription([
        global_sim_time, start_gazebo, global_clock_bridge, map_server, lifecycle_map,
        r1_sp, r1_br, r1_spawn, r1_ekf, r1_sm, r1_obs, r1_delayed,
        r2_sp, r2_br, r2_delayed, rviz
    ])