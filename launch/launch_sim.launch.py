import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
def generate_launch_description():
    package_name='mpcc' 
                    
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'True'}.items()
    )
                    
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'joint_broad'],
        output='screen'
    )

    load_diff_drive_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'diff_cont'],
        output='screen'
    )       
                    
                    
    # Run the spawner node from the gazebo_ros package.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                '-entity', 'mpcc'],
                        output='screen')
                        

    diff_drive_controller_node = Node(
        package='mpcc',
        executable='diff_drive_controller',
        name='diff_drive_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
                        
    guidance_controller_node = Node(
        package='mpcc',
        executable='guidance_controller',
        name='guidance_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    heading_controller_node = Node(
        package='mpcc',
        executable='heading_controller',
        name='heading_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    effort_diff_drive_controller_node = Node(
        package='mpcc',
        executable='differential_drive_effort',
        name='differential_drive_effort',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
                 
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_drive_base_controller],
            )
        ),

        rsp,
        gazebo,
        spawn_entity,
        # diff_drive_controller_node,
        # heading_controller_node,
        # guidance_controller_node,
    ])