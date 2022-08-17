import rclpy
from rclpy import node
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
import json
import os
from ament_index_python.packages import get_package_share_directory
from ws_controller.action_client import ToPoseClient
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import re

cmd_vel_pub = None
def main(args=None):
    rclpy.init(args=args)
    node = Node('my_node_name')
    global cmd_vel_pub
    cmd_vel_pub = node.create_publisher(Twist, "/cmd_vel", 4)
    sub_scan_front = node.create_subscription(LaserScan, 'scan_front', front_cb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    rclpy.spin(node)
    rclpy.shutdown()


def front_cb(data):
	forward = data.ranges[390]
	right = data.ranges[470]
	left = data.ranges[310]

	err_x = forward - 0.2
	err_z = left - right

	if err_x > 0.15:
		err_x = 0.15

	if err_x > -0.005 and err_x < 0.005:
		err_x = 0.0

	if err_z > -0.005 and err_z < 0.005:
		err_z = 0.0

	msg = Twist()
	msg.linear.x = err_x

	msg.angular.z = err_z
	global cmd_vel_pub
	cmd_vel_pub.publish(msg)



if __name__ == '__main__':
    main()