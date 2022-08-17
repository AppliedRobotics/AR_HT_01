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
class Controller():
    def __init__(self):
        self.node = Node('controller_ws')
        # self.sub_scan_left = self.node.create_subscription(LaserScan, 'scan_left', self.left_cb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # self.sub_scan_right = self.node.create_subscription(LaserScan, 'scan_right', self.right_cb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.sub_scan_front = self.node.create_subscription(LaserScan, 'scan_front', self.front_cb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.cmd_vel_pub = self.node.create_publisher(Twist, "/cmd_vel", 4)
        self.state_pub = self.node.create_publisher(String, "/state", 4)
        self.sub_state = self.node.create_subscription(String, 'command', self.command_cb, 10)
        path = os.path.join(get_package_share_directory('ws_controller'), 'params', 'points.json')
        with open(path, "r") as read_file:
            data = json.load(read_file)
        
        self.points_id = data['position_id']
        self.target_points = data['point_coordinate']
        
        print(self.target_points)
        self.action_client = ToPoseClient(self.node)
        self.node.create_timer(0.1, self.automat)
        self.state = 0#0 - do nothing, 1 - to first point, 2 - to second point, 3 - park first, 4 - park second, 
        #5 - parked first, 6 - parked second, 7 - to third point, 8 - park third, 9 - parked
        self.command = 0 #0 - do nothing, 1 - to first point, 2 - to second point
        self.old_command = 0
        self.y_target = 0.62
        self.x_target = 0.08
        self.kx = 3.0
        self.ky = 3.0
        self.kz = 7.0
    def automat(self):
        # print(self.action_client.get_feedback())
        # print(self.command)
        if self.command == 1:
            if self.state != 1 and self.state != 3 and self.state != 5:
                self.state = 1
                self.to_point(1)
                self.old_command = 1
                print("go to point 1")
            elif self.check_nav_state() == True:
                #statrt parking first
                print("reach point 1")
                self.state = 3
        elif self.command == 2:
            if self.state != 2 and self.state != 4 and self.state != 6:
                self.state = 2
                self.to_point(2)
                self.old_command = 2
                print("go to point 2")
            elif self.check_nav_state() == True:
                #statrt parking second
                print("reach point 2")
                self.state = 4
        elif self.command == 3:
            if self.state != 7 and self.state != 8:
                self.state = 7
                self.to_point(3) 
                self.old_command = 3
                print("go to point 3")
            elif self.check_nav_state() == True:
                print("reach point 3")
                self.state = 8

        msg = String()
        msg.data = str(self.state)
        self.state_pub.publish(msg)
        self.check_nav_state()
    def check_nav_state(self):
        feedback = self.action_client.get_feedback()
        print(feedback)
        if feedback["state"] == "goal reached":
            return True
        else:
            return False
    def front_cb(self,data):
        if self.state == 3 or self.state == 4 or self.state == 8: 
            right = data.ranges[90]
            left = data.ranges[270]
            if self.state != 8:
                y_err = self.ky*(self.y_target - right)
                z_err = self.kz*(data.ranges[150] - data.ranges[197])
                x_err = self.kx*(data.ranges[180]- self.x_target)
            else:
                y_err = self.ky*(0.4 - right)
                z_err = self.kz*(data.ranges[165] - data.ranges[190])
                x_err = self.kx*(data.ranges[180]- 0.4)
            print(x_err, y_err, z_err)
            x_err = self.check_constrain(x_err, 0.1)
            y_err = self.check_constrain(y_err, 0.1)
            z_err = self.check_constrain(z_err, 0.3)
         
            msg = Twist()
            msg.linear.x = x_err
            msg.linear.y = y_err
            msg.angular.z = z_err
            self.cmd_vel_pub.publish(msg)
            if(abs(x_err) < 0.06):
                x_err = 0
            if(abs(y_err) < 0.1):
                y_err = 0
            if(abs(x_err) < 0.06 and abs(y_err) < 0.1 and abs(z_err) < 0.05):
                if self.state == 8:
                    self.state = 9
                else:
                    self.state +=2
                self.command = 0
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.angular.z = 0.0
                self.cmd_vel_pub.publish(msg)
                print("parked")                
    def check_constrain(self, err, constr):
        if abs(err) > constr:
            if err > 0:
                return constr
            else:
                return -1*constr
            
        else:
            return err
    def to_point(self, point_num):
        x = self.target_points[point_num - 1][0]
        y = self.target_points[point_num - 1][1]
        z = self.target_points[point_num - 1][2]
        result = self.action_client.send_goal(x,y,z)
        # while self.action_client.get_feedback()['state'] != 'goal reached':
            # print(self.action_client.get_feedback())
    def command_cb(self, data):
        # print(data.data)
        try:
            f = data.data.split(":") 
            move_flag = int(re.sub(r'[^0-9]','',f[0]))
            zone = int(re.sub(r'[^0-9]','',f[1])) 
            if move_flag == 0:
                self.command = 0
            else:
                self.command = zone
        except Exception as e:
            print(e)
        # print(self.action_client.get_feedback())
def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller.node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
