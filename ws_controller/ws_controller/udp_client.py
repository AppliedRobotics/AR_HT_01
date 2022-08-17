# from setuptools import Command
# import rclpy
# from rclpy import node
# from rclpy.client import Client
# from rclpy.node import Node
# from rclpy.qos import ReliabilityPolicy, QoSProfile
# from std_msgs.msg import String
import json
import os
import socket
import re
import os
import json
import select
import _thread
import time

class client():#(Node):
    def __init__(self):
        # super().__init__('udp_client')
        # self.sub_state = self.create_subscription(String, 'state', self.state_cb, 10)
        # self.command_pub = self.create_publisher(String, "/command", 4)
        self.state = 'stay'
        self.zone = 0
        self.move_flag = 0
        
        home = os.path.expanduser("~")
        path = home+"/config.json"
        with open(path, "r") as read_file:
            data = json.load(read_file)
        self.UDP_PORT_IN = 8888
        self.UDP_PORT_OUT = 9090
        self.UDP_SERVER_ADDRESS = data["robot_adress"] #rp adress
        self.UDP_CLIENT_ADDRESS = data["server_adress"] #pc adress
        print(self.UDP_SERVER_ADDRESS)
        print(self.UDP_CLIENT_ADDRESS)
        self.MAX_UDP_PACKET=128 # max size of incoming packet to avoid sending too much data to the micro
        self.ROBOT_ID = data["robot_id"]
        self.udp_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        server_adress = ('', self.UDP_PORT_IN)
        print("server adress: ",server_adress)
        self.udp_socket.bind(server_adress) 
        print(self.connect_udp())
        _thread.start_new_thread(self.udp_receive, ())
        # _thread.start_new_thread(self.send_state, ())
        
    def send_state(self):
        while True:
            payload = "S:"+self.ROBOT_ID+":"+str(self.state)+":"+str(self.zone)+"#\n"
            self.udp_socket.sendto(payload.encode(), (self.UDP_CLIENT_ADDRESS, self.UDP_PORT_OUT))
            # print(payload)
            time.sleep(0.1)

    def udp_receive(self):
        while True:
            print("read")
            (rlist, wlist, xlist) = select.select([self.udp_socket], [], []) 
            print('read2')
            if self.udp_socket in rlist:
                data, udp_client = self.udp_socket.recvfrom(self.MAX_UDP_PACKET, )
                self.messageHandler(data.decode('utf-8'))
                time.sleep(0.01)

    def messageHandler(self, msg):
        command_msg = String()

        if msg != None and len(msg)!=0:
            print(msg)
            if msg[0] == 's':
                msg_split = msg.split(':')
                self.move_flag = msg_split[1]
                self.zone = msg_split[2]
                
                if int(self.zone) == 0:
                    zone = 200
                command_msg.data = str(self.move_flag)+":"+str(self.zone)
                self.command_pub.publish(command_msg)

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
    #rclpy.init(args=args)
    Client = client()
    #rclpy.spin(Client)
    #rclpy.shutdown()
    try:
        while True:
            time.sleep(0.01)
    except Exception as e:
        print(e)
    
if __name__ == '__main__':
    main()
