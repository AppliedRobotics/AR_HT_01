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
        super().__init__('udp_client')
        self.sub_state = self.create_subscription(String, 'state', self.state_cb, 10)
        self.command_pub = self.create_publisher(String, "/command", 4)
        self.state = 'stay'
        self.zone = 0
        self.move_flag = 0
        
        home = os.path.expanduser("~")
        path = home+"/config.json"
        with open(path, "r") as read_file:
            data = json.load(read_file)
        self.UDP_PORT_IN = 8888 
        self.UDP_PORT_OUT = 8888
        self.UDP_SERVER_ADDRESS = data["robot_adress"] #rp adress
        self.UDP_CLIENT_ADDRESS = data["server_adress"] #pc adress
        self.MAX_UDP_PACKET=128 # max size of incoming packet to avoid sending too much data to the micro
        self.ROBOT_ID = data["robot_id"]
        self.udp_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.udp_socket.bind((self.UDP_SERVER_ADDRESS, self.UDP_PORT_IN)) 
        print(self.connect_udp())
        self.create_timer(0.05, self.udp_receive)
        self.create_timer(1.0, self.send_state)
        
    def send_state(self):
        payload = "S:"+self.ROBOT_ID+":"+str(self.state)+":"+str(self.zone)+"#\n"
        self.udp_socket.sendto(payload.encode(), (self.UDP_CLIENT_ADDRESS, self.UDP_PORT_OUT))
    def udp_receive(self):
        command_msg = String()
        try:
            data, udp_client = self.udp_socket.recvfrom(self.MAX_UDP_PACKET, )  
            self.parser(data.decode())
            if int(self.zone) == 0:
                zone = 200
            command_msg.data = str(self.move_flag)+":"+str(self.zone)
            self.command_pub.publish(command_msg)
        except Exception as e:
            if not str(e) == 'timed out':
                print("parsing error, receive: " + str(data) +" "+ str(e))
    def parser(self, str): 
        f = str.split(":") 
        if(f[0] == "s"): 
            self.move_flag = int(re.sub(r'[^0-9]','',f[1]))
            self.zone = int(re.sub(r'[^0-9]','',f[2])) 
            # print(move_flag," ", zone) 

    def connect_udp(self):
        try: 
            self.udp_socket.connect((self.UDP_CLIENT_ADDRESS, self.UDP_PORT_OUT)) 
            print('connection succesful')
            self.udp_socket.settimeout(0.5) 
            return True
        except Exception as e: 
            print('connection failed', e)
        return False

    def state_cb(self,data):
        self.state = data.data

def main(args=None):
    rclpy.init(args=args)
    Client = client()
    rclpy.spin(Client)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()