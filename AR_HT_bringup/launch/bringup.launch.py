from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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