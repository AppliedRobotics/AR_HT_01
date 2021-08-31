from time import sleep, time
from math import pi, sin, cos
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion

class WheelOdom(Node):
    def __init__(self):
        super().__init__('wheel_odom')
        self.sub = self.create_subscription(
            JointState, 'joint_states', self.js_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.odom_broadcaster = TransformBroadcaster(self)
        self.BASELINE = 0.430
        self.WHEELRADIUS = 0.115
        self.x = 0
        self.y = 0
        self.theta = 0
        self.xold = 0
        self.yold = 0
        self.timeold = self.get_clock().now().nanoseconds
        print('odometry initialized')
    def js_cb(self, data):
        now = self.get_clock().now()
        time = now.nanoseconds
        delta = time - self.timeold
        if delta/(10**9) >= 0.02:
            current_time = time/(10**9)
            self.timeold = time
            Vx, Vy, Vtheta = self.calculate_odom(delta/(10**9), data.velocity[1], data.velocity[0])
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.theta / 2)
            quaternion.w = cos(self.theta / 2)
            transform_stamped_msg = TransformStamped()
            transform_stamped_msg.header.stamp = now.to_msg()
            transform_stamped_msg.header.frame_id = 'odom'
            transform_stamped_msg.child_frame_id = 'base_link'
            transform_stamped_msg.transform.translation.x = self.x
            transform_stamped_msg.transform.translation.y = self.y
            transform_stamped_msg.transform.translation.z = 0.0
            transform_stamped_msg.transform.rotation.x = quaternion.x
            transform_stamped_msg.transform.rotation.y = quaternion.y
            transform_stamped_msg.transform.rotation.z = quaternion.z
            transform_stamped_msg.transform.rotation.w = quaternion.w
            self.odom_broadcaster.sendTransform(transform_stamped_msg)
            q = self.quaternion_from_euler(0.0,0.0, 0.0)
            quaternion.x = q[1]
            quaternion.y = q[2]
            quaternion.z = q[3]
            quaternion.w = q[0]
            transform_stamped_msg = TransformStamped()
            transform_stamped_msg.header.stamp = now.to_msg()
            transform_stamped_msg.header.frame_id = 'base_link'
            transform_stamped_msg.child_frame_id = 'laser_hls'
            transform_stamped_msg.transform.translation.x = -0.350
            transform_stamped_msg.transform.translation.y = 0.0
            transform_stamped_msg.transform.translation.z = 0.4
            transform_stamped_msg.transform.rotation.x = quaternion.x
            transform_stamped_msg.transform.rotation.y = quaternion.y
            transform_stamped_msg.transform.rotation.z = quaternion.z
            transform_stamped_msg.transform.rotation.w = quaternion.w
            self.odom_broadcaster.sendTransform(transform_stamped_msg)
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.theta / 2)
            quaternion.w = cos(self.theta / 2)
            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = 'odom'
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = self.theta
            odom.pose.covariance[0] = 0.1
            odom.pose.covariance[7] = 0.1
            odom.pose.covariance[35] = 0.1
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = 'base_link'
            odom.twist.twist.linear.x = Vx
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = Vtheta
            odom.twist.covariance[0] = 0.01
            odom.twist.covariance[7] = 0.01
            odom.twist.covariance[35] = 0.01
            self.odom_pub.publish(odom)
           
    def quaternion_from_euler(self, roll, pitch, yaw):    
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr
        return q

    def calculate_odom(self, delta, Lvel, Rvel):
        # Lvel= -Lvel
        vel = (self.WHEELRADIUS)*(Rvel+Lvel)/2
        # to calc prev speed we need theta old(В локальной ск у нашего робота есть только 1 составляющая скорости)
        Vx = vel*sin(pi/2)
        
        Vy = 0
        # New theta for caclulationg rotation matrix:
        Vtheta = (self.WHEELRADIUS)*(Lvel-Rvel)/self.BASELINE
        # Rotation matrix
        self.theta += delta * Vtheta
        self.y += delta * sin(self.theta)*Vx
        self.x += delta * cos(self.theta)*Vx
        return Vx, Vy, Vtheta


def main():
    rclpy.init()
    odom = WheelOdom()
    rclpy.spin(odom)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
