import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    remappings_hls_right = [('/scan', '/scan_right'),]
    remappings_hls_left = [('/scan', '/scan_left'),]
    remappings_hokuyo = [('/scan', '/scan_front_wrong'),]

    package_dir = get_package_share_directory('AR_HT_bringup')
    config = os.path.join(
        package_dir,
        'config',
        'hokuyo.yaml'
        )
    return LaunchDescription([
        Node(
            package='urg_node', 
            executable='urg_node_driver',
            name='urg_node',
            output='screen',
            parameters=[config],
            remappings=remappings_hokuyo
            ),
        Node(
            package='AR_HT_bringup',
            executable='scan_fixer',
            name='scan_fixer_node',
           output='screen',
            emulate_tty=True,
        ),
        Node(
            package='AR_HT_bringup',
            executable='omni_control',
            name='omni_control_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='AR_HT_bringup',
            executable='odom',
            name='odom_node',
            output='screen',
            emulate_tty=True,
        ),
        # Node(
        #     package='AR_HT_bringup',
        #     executable='serial_connection',
        #     name='serial_node',
        #     output='screen',
        #     emulate_tty=True,
        # ),
        # Node(
        #     package='AR_HT_bringup',
        #     executable='fixer',
        #     name='scan_fixer',
        #     output='screen',
        #     emulate_tty=True,
        # ),  
    ])
    # return LaunchDescription([
    #     Node(
    #         package='AR_HT_bringup',
    #         executable='serial_connection',
    #         name='serial_node',
    #         output='screen',
    #         emulate_tty=True,
    #         parameters=[
    #             {'usb': '/dev/ttyACM0'}
    #         ]
    #     )
    # ])
    
