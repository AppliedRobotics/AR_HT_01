import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import random
import math
import time
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
class OdomNoise(Node):
    def __init__(self):
        super().__init__('odom_noise')
        self.sub = self.create_subscription(Odometry, 'odom_ideal', self.odom_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        random.seed()
        self.alpha_0 = 0.5
        self.alpha_1 = 0.5
        self.alpha_2 = 0.5
        self.alpha_3 = 0.5
        self.alpha_4 = 0.3
        self.alpha_5 = 0.3
        self.x = 0
        self.y = 0 
        self.theta = 0
        self.first_msg_flag = True
        self.current_time = time.time()
        self.odom_broadcaster = TransformBroadcaster(self)
    def odom_cb(self,data):
        if self.first_msg_flag == False:
            delta = time.time() - self.current_time
            self.current_time = time.time()
            # v_clean = round(data.twist.twist.linear.x, 3)
            # w_clean = round(data.twist.twist.angular.z, 3)
            vel_x = data.twist.twist.linear.x + self.get_random_vel(data.twist.twist.linear.x, data.twist.twist.angular.z, self.alpha_0, self.alpha_1)
            w_z = data.twist.twist.angular.z + self.get_random_vel(data.twist.twist.angular.x, data.twist.twist.angular.z, self.alpha_1, self.alpha_2)
            gamma = self.get_random_vel(data.twist.twist.angular.x, data.twist.twist.angular.z, self.alpha_4, self.alpha_5)
            self.theta += w_z*delta + gamma*delta
            self.x = self.x + math.cos(self.theta)* vel_x * delta
            self.y = self.y + math.sin(self.theta)* vel_x * delta
            self.send_tf_and_odom(vel_x, w_z, data.header.stamp)
        else:
            self.first_msg_flag = False
            self.x = data.pose.pose.position.x 
            self.y = data.pose.pose.position.y
            _, _, self.theta = self.euler_from_quaternion(data.pose.pose.orientation.x,
             data.pose.pose.orientation.y,
             data.pose.pose.orientation.z,
             data.pose.pose.orientation.w)
    def send_tf_and_odom(self, vel_x, w_z, _time_ = 0):
        # if _time_ == 0.0:
            # _time_ = self.get_clock().now().to_msg()
        # print(_time_)
        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.stamp = _time_ 
        transform_stamped_msg.header.frame_id = 'odom'
        transform_stamped_msg.child_frame_id = 'base_link'
        transform_stamped_msg.transform.translation.x = self.x
        transform_stamped_msg.transform.translation.y = self.y
        transform_stamped_msg.transform.translation.z = 0.0
        transform_stamped_msg.transform.rotation.x = 0.0
        transform_stamped_msg.transform.rotation.y = 0.0
        transform_stamped_msg.transform.rotation.z = math.sin(self.theta / 2)
        transform_stamped_msg.transform.rotation.w = math.cos(self.theta / 2)
        self.odom_broadcaster.sendTransform(transform_stamped_msg)
        transform_stamped_msg.header.stamp = _time_ 
        transform_stamped_msg.header.frame_id = 'base_link'
        transform_stamped_msg.child_frame_id = 'scan_1'
        transform_stamped_msg.transform.translation.x = 0.102
        transform_stamped_msg.transform.translation.y = 0.0
        transform_stamped_msg.transform.translation.z = 0.183
        transform_stamped_msg.transform.rotation.x = 0.0
        transform_stamped_msg.transform.rotation.y = 0.0
        transform_stamped_msg.transform.rotation.z = 0.0
        transform_stamped_msg.transform.rotation.w = 1.0
        self.odom_broadcaster.sendTransform(transform_stamped_msg)
        transform_stamped_msg.header.stamp = _time_ 
        transform_stamped_msg.header.frame_id = 'base_link'
        transform_stamped_msg.child_frame_id = 'scan_2'
        transform_stamped_msg.transform.translation.x = -0.3335
        transform_stamped_msg.transform.translation.y = 0.0
        transform_stamped_msg.transform.translation.z = 0.183
        transform_stamped_msg.transform.rotation.x = 0.0
        transform_stamped_msg.transform.rotation.y = 0.0
        transform_stamped_msg.transform.rotation.z = 0.0
        transform_stamped_msg.transform.rotation.w = 1.0
        self.odom_broadcaster.sendTransform(transform_stamped_msg)
        
        msg = Odometry()
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        msg.pose.pose.orientation.w = math.cos(self.theta / 2)
        msg.twist.twist.linear.x = vel_x
        msg.twist.twist.angular.z = w_z
        self.odom_pub.publish(msg)
    def get_random_vel(self, vel, w, alpha_0, alpha_1):
        sigma = vel**2 * alpha_0 + w**2 * alpha_1
        rand_sum = 0
        for i in range (0,12):
            rand_sum += random.gauss(0, sigma)
        err = rand_sum/2
        return err
    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
    
        return roll_x, pitch_y, yaw_z # in radians
def main():
	rclpy.init()
	noiser = OdomNoise()
	rclpy.spin(noiser)
	rclpy.shutdown()
if __name__ == '__main__':
    main()
