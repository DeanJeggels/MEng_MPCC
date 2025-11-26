# Copyright (c) 2020 Samsung Research Russia
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Modified by robotics.snowcron.com


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

import sys
sys.path.append("src/mpcc/launch") 
from globals import *

def generate_launch_description():
    # Input parameters declaration
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')


    # Variables


    # Now in globals.py
    # package_name = 'navigation_bot_05'
    # bringup_dir = get_package_share_directory(package_name)

    # slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    # slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_sync_launch.py')

    # Create our own temporary YAML files that include substitutions

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')


    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    # Nodes launching commands
    # start_slam_toolbox_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(slam_launch_file),
    #     launch_arguments={'use_sim_time': use_sim_time}.items())

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        # arguments=['-d'],
        parameters=[os.path.join(bringup_dir, 'config/rtabmap_params.yaml')],
        # remappings=[('/rtabmap/odom', '/odom'),  
        #              ('/rtabmap/map', '/map')]
        # remappings=[('/odom', '/diff_cont/odom')],
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Running SLAM Toolbox
    # ld.add_action(start_slam_toolbox_cmd)
    ld.add_action(rtabmap_node)

    # Running Map Saver Server

    return ld
