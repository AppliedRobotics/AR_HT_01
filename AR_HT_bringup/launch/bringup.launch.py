import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    remappings_hls = [('/scan', '/scan'),]
    package_dir = get_package_share_directory('AR_HT_bringup')
    config = os.path.join(
        package_dir,
        'config',
        'hokuyo.yaml'
        )
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000') #for A3 is 256000
    frame_id = LaunchConfiguration('frame_id', default='laser_hls')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),
        Node(
            package='rplidar_ros2',
            executable='rplidar_scan_publisher',
            name='rplidar_scan_publisher',
            parameters=[{'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted, 
                         'angle_compensate': angle_compensate, 
                         'scan_mode': scan_mode}],
            output='screen'),
        Node(
            package='AR_HT_bringup',
            executable='dif_control',
            name='dif_control_node',
            output='screen',
            emulate_tty=True,
        ),
        # Node(
        #     package='AR_HT_bringup',
        #     executable='serial_connection',
        #     name='opencr_node',
        #     output='screen',
        #     emulate_tty=True,
        #     parameters=[
        #         {'usb': '/dev/ttyACM0'}
        #     ]
        # ),
        Node(
            package='AR_HT_bringup',
            executable='odom',
            name='odom_node',
            output='screen',
            emulate_tty=True,
        ),
        # Node(
        #     package='AR_HT_bringup',
        #     executable='ekf',
        #     name='ekf_node',
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
    
