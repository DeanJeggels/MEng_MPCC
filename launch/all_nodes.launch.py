#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Karretjie node: propagates and publishes odometry
        # Node(
        #     package='mpcc',
        #     executable='karretjie_opt',
        #     name='karretjie',
        #     output='screen'
        # ),
          # 3. Trajectory visualizer: plots actual vs. reference path and time-series data

        Node(
            package='mpcc',
            executable='trajectory_viz.py',
            name='optimal_sequence_visualizer',
            output='screen'
        ),
        # 2. MPCC controller: runs the model predictive contour controller
        Node(
            package='mpcc',
            executable='mpcc_controller_opt',
            name='mpcc_controller',
            output='screen'
        ),

      
    ])
