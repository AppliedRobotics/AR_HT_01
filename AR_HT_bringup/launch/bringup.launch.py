import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hls_lfcd_lds_driver',
            node_executable='hlds_laser_publisher',
            node_name='hlds_laser_publisher',
            parameters=[{'port': '/dev/ttyUSB0', 'frame_id': 'laser'}],
            remappings=[('scan', 'scan_hls')]
            output='screen'),
    ])
    return LaunchDescription([
        Node(
            package='AR_HT_bringup',
            executable='dif_control',
            name='dif_control_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'usb': '/dev/ttyUSB0'}
            ]
        )
    ])
    return LaunchDescription([
        Node(
            package='AR_HT_bringup',
            executable='serial_connection',
            name='serial_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'usb': '/dev/ttyACM0'}
            ]
        )
    ])