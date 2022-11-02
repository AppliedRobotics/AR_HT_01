import rclpy
from rclpy.node import Node
import threading
from nav_msgs.msg import OccupancyGrid
import numpy as np
import tf2_ros
from tf2_msgs.msg import TFMessage
from math import asin, sin, cos, pi, atan2, isnan
from sensor_msgs.msg import LaserScan, JointState
import subprocess
from nav_msgs.msg import Path, Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import json
from std_msgs.msg import Bool, Float32
from time import sleep, time
class AR_HT_api(Node):
    def __init__(self):
        super().__init__('ar_api')
        self.subscription_1 = self.create_subscription(OccupancyGrid,'map',self.map_callback,10)
        self.subscription_2 = self.create_subscription(TFMessage,'tf',self.tf_callback,10)
        self.subscription_3 = self.create_subscription(LaserScan,'scan_1',self.scan_callback_1,1)
        self.subscription_4 = self.create_subscription(LaserScan,'scan_2',self.scan_callback_2, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription_5 = self.create_subscription(OccupancyGrid,'keepout_filter_mask',self.keepout_callback,10)
        self.subscription_6 = self.create_subscription(Path,'plan',self.plan_callback,10)
        self.subscription_7 = self.create_subscription(JointState,'joint_states',self.js_callback,10)
        self.subscription_8 = self.create_subscription(Float32,'battery_voltage',self.battery_callback,10)
        self.create_timer(3.0, self.timer_callback)
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.lift_pub = self.create_publisher(Bool, 'lift', 1)
        self.map_ = None
        self.keepout_map = None
        self.tf_time = 0
        self.scan_buffer_1 = [0]*100
        self.scan_angles_1 = [0]*100
        self.scan_buffer_2 = [0]*100
        self.scan_angles_2 = [0]*100
        self.scan_flag_1 = False
        self.scan_flag_2 = False
        self.plan = []
        self.plan_flag = False
        self.js_flag = False
        self.battery_voltage = 0.0
    def battery_callback(self, data):
        self.battery_voltage = data.data
    def js_callback(self, data):
        self.js_flag = True
    def map_callback(self, data):
        self.map_ = data
    def keepout_callback(self, data):
        self.keepout_map = data
    def timer_callback(self):
        self.js_flag = False
        self.battery_voltage = False
        self.scan_flag_2 = False
        self.scan_flag_1 = False
        self.plan_flag = False
    def tf_callback(self, tf):
        if tf.transforms[0].child_frame_id == 'base_link':
            self.tf_time = tf.transforms[0].header.stamp
    def scan_callback_1(self, data):
        increment = int(len(data.ranges)/len(self.scan_buffer_1))                
        j = 0
        for i in range(0, len(self.scan_buffer_1)*increment, increment):
            self.scan_buffer_1[j] = data.ranges[i]
            self.scan_angles_1[j] = i*data.angle_increment + data.angle_min
            j+=1
        self.scan_flag_1 = True
    def scan_callback_2(self, data):
        increment = int(len(data.ranges)/len(self.scan_buffer_2))                
        j = 0
        for i in range(0, len(self.scan_buffer_2)*increment, increment):
            self.scan_buffer_2[j] = data.ranges[i]
            self.scan_angles_2[j] = i*data.angle_increment + data.angle_min
            j+=1
        self.scan_flag_2 = True
    def plan_callback(self, data):
        self.plan = []
        for pose in plan.poses:
            x = pose.pose.position.x 
            y = pose.pose.position.y
            self.plan.append([x,y])
    def get_map(self):
        if self.keepout_map is not None:
            map_loc = self.keepout_map 
        elif self.map_ is not None:
            map_loc = self.map_
        else:
            return False
        blank_image = np.zeros((map_loc.info.height, map_loc.info.width,1), np.uint8)
        for i in range(0, map_loc.info.height):
            for j in range(0, map_loc.info.width):
                k = j + (map_loc.info.height - i - 1)*map_loc.info.width
                if map_loc.data[int(k)] == 0:
                    blank_image[i][j] = 254
                elif map_loc.data[int(k)] == 100:
                    blank_image[i][j] = 0
                else:
                    blank_image[i][j] = 200
        answr = {'map': blank_image, 
                'map_origin': (map_loc.info.origin.position.x, map_loc.info.origin.position.y, 0),
                'map_width': map_loc.info.width,
                'map_height': map_loc.info.height}
        return answr
    def get_scans(self):
        if self.scan_flag_1 == True and self.scan_flag_2 == True:
            answr = {'scan_1_ranges': self.scan_buffer_1, 'scan_1_angles': self.scan_angles_1,
            'scan_2_ranges': self.scan_buffer_2, 'scan_2_angles': self.scan_angles_2}
            return answr  
        else:
            return False
    def get_robot_pose(self):
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', self.tf_time)
            translation = trans.transform.translation 
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            robot_angle = atan2(siny_cosp, cosy_cosp)+pi/2
            answr = {'robot_pose': (translation.x, translation.y, robot_angle)}
            return answr
        except Exception as e: 
            return False
    def get_plan(self):
        if self.plan_flag == True:
            answr = {'plan': self.plan}
            return answr
        else:
            return False
    def get_status(self):
        if self.battery_voltage == 0:
            battery = False
        else:
            battery = self.battery_voltage
        answr = {'motors': self.js_flag,
                'scan_1': self.scan_flag_1,
                'scan_2': self.scan_flag_2,
                'lift': True,
                'battery': battery}

    def lift(self, state, sleep_time=0): #sleep time = 12 for full cycle of up and down
        msg = Bool()
        if state == True:
            msg.data = True
            self.lift_pub.publish(msg)
            sleep(sleep_time)
            return True
        elif state == False:
            msg.data = False
            self.lift_pub.publish(msg)
            sleep(sleep_time)
            return True
        return False

  
def main(args=None):
    rclpy.init(args=args)
    api = AR_HT_api()
    thread = threading.Thread(target=rclpy.spin, args=(api, ), daemon=True)
    thread.start()
    rate = api.create_rate(2)
    # future = action_client.send_goal(-1.85,-1.12,5.1)
    while rclpy.ok():
        try:
            api.get_map()
            for i in range(0,1):
                print()
            rate.sleep()
        except KeyboardInterrupt:
            rclpy.shutdown()



if __name__ == '__main__':
    main()
