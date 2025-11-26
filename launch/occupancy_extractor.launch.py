from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'occupancy_threshold',
            default_value='50',
            description='Threshold for occupied cells (0-100)'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='map',
            description='Frame ID for published messages'
        ),
        
        Node(
            package='occupancy_extractor',
            executable='occupancy_extractor_node',
            name='occupancy_extractor',
            parameters=[{
                'occupancy_threshold': LaunchConfiguration('occupancy_threshold'),
                'frame_id': LaunchConfiguration('frame_id')
            }],
            output='screen'
        )
    ])
