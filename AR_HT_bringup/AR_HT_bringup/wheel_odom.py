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
        self.odom_pub = self.create_publisher(Odometry, "odom_dirty", 10)
        self.odom_broadcaster = TransformBroadcaster(self)
        self.RADIUS = 0.2
        L1 = 0.3
        L2 = 0.4
        self.WHEEL_SEPARATION = (L1 + L2)/2
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
            Vx, Vy, Vtheta = self.calculate_odom(delta/(10**9), data.velocity[0], data.velocity[1], data.velocity[2], data.velocity[3])
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
            # quaternion.x = 0.0
            # quaternion.y = 0.0
            # quaternion.z = 0.0
            # quaternion.w = 1.0
            # transform_stamped_msg = TransformStamped()
            # transform_stamped_msg.header.stamp = now.to_msg()
            # transform_stamped_msg.header.frame_id = 'base_link'
            # transform_stamped_msg.child_frame_id = 'laser_hokuyo'
            # transform_stamped_msg.transform.translation.x = 0.150
            # transform_stamped_msg.transform.translation.y = 0.0
            # transform_stamped_msg.transform.translation.z = 0.4
            # transform_stamped_msg.transform.rotation.x = quaternion.x
            # transform_stamped_msg.transform.rotation.y = quaternion.y
            # transform_stamped_msg.transform.rotation.z = quaternion.z
            # transform_stamped_msg.transform.rotation.w = quaternion.w
            # self.odom_broadcaster.sendTransform(transform_stamped_msg)
            # quaternion.x = 0.0
            # quaternion.y = 0.0
            # quaternion.z = 0.0
            # quaternion.w = 1.0
            # transform_stamped_msg = TransformStamped()
            # transform_stamped_msg.header.stamp = now.to_msg()
            # transform_stamped_msg.header.frame_id = 'base_link'
            # transform_stamped_msg.child_frame_id = 'laser_hls'
            # transform_stamped_msg.transform.translation.x = -0.350
            # transform_stamped_msg.transform.translation.y = 0.0
            # transform_stamped_msg.transform.translation.z = 0.4
            # transform_stamped_msg.transform.rotation.x = quaternion.x
            # transform_stamped_msg.transform.rotation.y = quaternion.y
            # transform_stamped_msg.transform.rotation.z = quaternion.z
            # transform_stamped_msg.transform.rotation.w = quaternion.w
            # self.odom_broadcaster.sendTransform(transform_stamped_msg)
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
            odom.twist.twist.linear.y = Vy
            odom.twist.twist.angular.z = Vtheta
            odom.twist.covariance[0] = 0.01
            odom.twist.covariance[7] = 0.01
            odom.twist.covariance[35] = 0.01
            self.odom_pub.publish(odom)
           

    def calculate_odom(self, delta, v_lf, v_rf, v_lb, v_rb):
        # Lvel= -Lvel
        # to calc prev speed we need theta old(В локальной ск у нашего робота есть только 1 составляющая скорости)
        # Vx=(vel0+vel1-vel2-vel3)*(R/4)
        # Vy=(-vel0-vel2+vel3+vel1)*(R/4)
        # Vtheta=(-vel0+vel2-vel3+vel1)*(R/(4*wheel_separation))
        
        Vx = (v_lf + v_rf + v_lb + v_rb) * (self.RADIUS/4) * 0.159154943
        Vy = (-v_lf + v_rf + v_lb - v_rb) * (self.RADIUS/4) * 0.159154943
        Vtheta = (-v_lf + v_rf - v_lb + v_rb) * (self.RADIUS/(4*self.WHEEL_SEPARATION)) * 0.159154943
        # Vtheta = 0.0
        # print(round(Vx,3), round(Vy,3), round(Vtheta,3))
        # print(v_lf-v_rf, v_lb-v_rb)
        # print(round(v_lf,3), round(v_rf,3), round(v_lb,3), round(v_rb,3))
        # Rotation matrix
        self.theta += delta * Vtheta
        self.y += delta * (sin(self.theta)*Vx + cos(self.theta)*Vy)
        self.x += delta * (cos(self.theta)*Vx - sin(self.theta)*Vy)
        return Vx, Vy, Vtheta


def main():
    rclpy.init()
    odom = WheelOdom()
    rclpy.spin(odom)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
