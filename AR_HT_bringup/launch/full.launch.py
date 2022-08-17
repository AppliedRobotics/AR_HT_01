import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    base_pkg = get_package_share_directory('AR_HT_bringup')
    start_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_pkg, 'bringup.launch.py'),
            )
        )
    nav_pkg = get_package_share_directory('AR_HT_navigation')
    start_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'full.launch.py'),
            )
        )
    controller = Node(
                package='ws_controller',
                executable='controller',
                name='main_controller',
                output='screen',
                emulate_tty=True)
    return LaunchDescription([
        start_base,
        #start_nav,
        controller       
    ])
    
