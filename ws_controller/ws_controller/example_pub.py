from setuptools import Command
import rclpy
from rclpy import node
from rclpy.client import Client
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import String
import json
import os
import socket
import re
import os
import json
class client(Node):
    def __init__(self):
        super().__init__('example_pub')
        self.sub_state = self.create_subscription(String, 'state', self.state_cb, 10)
        self.command_pub = self.create_publisher(String, "/command", 4)
        self.create_timer(1.0, self.timer_cb)
        self.state = 3

    def state_cb(self,data):
        a = 0
        self.state = int(data.data)
        print(self.state)

    def timer_cb(self):
        msg = String()
        # if self.state == 0:
        #     msg.data = "1:2"
        # if self.state == 5:
        #     msg.data = "1:2"
        # if self.state == 8:
        #     msg.data = "1:1"
        # if self.state == 6:
        msg.data = "1:3"
        self.command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    Client = client()
    rclpy.spin(Client)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
