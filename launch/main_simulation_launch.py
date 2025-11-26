# bringup_dir/launch/mpcc_simulation_launch.py

from launch import LaunchDescription
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

import os
import xacro
import sys
sys.path.append("src/mpcc/launch")
from globals import *

def generate_launch_description():
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world', default=world_path)

    declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')
    declare_use_namespace_cmd = DeclareLaunchArgument('use_namespace', default_value='false', description='Whether to apply a namespace')
    declare_slam_cmd = DeclareLaunchArgument('slam', default_value='True', description='Enable SLAM')
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation clock')
    declare_rtabmap_params_file_cmd = DeclareLaunchArgument('rtabmap_params_file', default_value=rtabmap_params_path, description='RTAB-Map params file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument('rviz_config_file', default_value=rviz_path, description='RViz config file')
    declare_use_simulator_cmd = DeclareLaunchArgument('use_simulator', default_value='True', description='Start simulator')
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument('use_robot_state_pub', default_value='True', description='Start robot_state_publisher')
    declare_use_rviz_cmd = DeclareLaunchArgument('use_rviz', default_value='True', description='Start RViz')
    declare_simulator_cmd = DeclareLaunchArgument('headless', default_value='False', description='Run gzclient')
    declare_world_cmd = DeclareLaunchArgument('world', default_value=world_path, description='World file')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'gui':'True', 'server':'True'}.items(),
    )

    robot_description_config = xacro.process_file(urdf)
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[params]
    )

    # spawn_entity_cmd = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=['-topic', 'robot_description',
    #                '-x', '0.222705', '-y', '-0.162081', '-z', '0.0',
    #                '-Y', '0.3700960527801269', '-R', '0.0', '-P', '0.0',
    #                '-entity', 'mpcc'],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen'
    # )


    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '0.0',
                   '-Y', '0.3', '-R', '0.0', '-P', '0.0',
                # '-Y', '0.0', '-R', '0.0', '-P', '0.0',
                   '-entity', 'mpcc'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

        # Transform point cloud node
    transform_cloud_node = Node(
        package='mpcc',  # Replace with your package
        executable='frame_transform.py',
        name='ouster_transformer',
        output='screen',
        parameters=[{
            'target_frame': 'base_link',
            'timeout': 0.2
        }]
    )

    
    # 3D LiDAR ICP Odometry (scan_cloud -> odom + TF)
    icp_odometry_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[os.path.join(bringup_dir, 'config/icp_odometry.yaml')],
        remappings=[
            ('scan_cloud', '/scan_cloud'),
        ],
    )

    # RTAB-Map SLAM (fresh DB each run)
    # rtabmap_node = Node(
    #     package='rtabmap_slam',
    #     executable='rtabmap',
    #     name='rtabmap',
    #     output='screen',
    #     parameters=[os.path.join(bringup_dir, 'config/rtabmap_params.yaml')],
    #     remappings=[
    #         ('scan_cloud', '/scan_cloud'),
    #         ('odom', '/odom'),
    #     ],
    #     arguments=['--delete_db_on_start'],
    # )

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[os.path.join(bringup_dir, 'config/rtabmap_params.yaml')],
        remappings=[
            ('scan_cloud', '/scan_cloud'),
            ('odom', '/odom'),
        ],
    )

    # Optional: bringup/nav2 and rviz
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'use_sim_time': use_sim_time}.items())
    bringup_timer_action = launch.actions.TimerAction(period=5.0, actions=[bringup_cmd])

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': '',
                          'use_namespace': 'False',
                          'rviz_config': rviz_config_file}.items())
    rviz_timer_action = launch.actions.TimerAction(period=3.0, actions=[rviz_cmd])

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_broad'],
        output='screen'
    )

    load_diff_drive_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_cont'],
    )

    # Timed bring-up to ensure odometry is ready before SLAM
    icp_timer_action = launch.actions.TimerAction(period=5.0, actions=[icp_odometry_node])
    rtabmap_timer_action = launch.actions.TimerAction(period=8.0, actions=[rtabmap_node])

    ld = LaunchDescription()
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rtabmap_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)

    ld.add_action(gazebo)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(spawn_entity_cmd)

    # ld.add_action(transform_cloud_node)
    ld.add_action(icp_timer_action)
    ld.add_action(rtabmap_timer_action)
    ld.add_action(rviz_timer_action)

    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_cmd,
            on_exit=[load_joint_state_controller],
        )
    ))
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_diff_drive_base_controller],
        )
    ))

    # ld.add_action(bringup_timer_action)
    return ld
