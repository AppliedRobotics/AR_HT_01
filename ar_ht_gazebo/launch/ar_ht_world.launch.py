#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'caffe_world.world'
    world = os.path.join(get_package_share_directory('ar_ht_gazebo'),
                         'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('ar_ht_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    return LaunchDescription([
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),
        Node(
            package='gazebo_noiser',
            executable='noise_odom',
            name='noise_odom_node',
            output='log',
            emulate_tty=True,
        ),
    ])