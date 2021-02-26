import rclpy
from rclpy.node import Node
import threading
from nav_msgs.msg import OccupancyGrid, Odometry
import tf2_ros
import numpy as np
from tf2_msgs.msg import TFMessage
from copy import copy
from math import asin, sin, cos, pi, atan2
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
class EKF(Node):
    def __init__(self):
        super().__init__('ekf_node')
        self.subscription_1 = self.create_subscription(Odometry,'odom_dirty',self.odom_callback,10)
        self.subscription_2 = self.create_subscription(TFMessage,'tf',self.tf_callback,10)
        self.subscription_3 = self.create_subscription(Twist,'cmd_vel',self.cmd_callback,10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.odom_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.05, self.main_cicle)
        np.set_printoptions(precision=3,suppress=True)
        self.A_k_minus_1 = np.array([[1.0,  0,   0, 0, 0],
                                                [  0,1.0,   0, 0, 0],
                                                [  0,  0, 1.0, 0, 0],
                                                [  0,  0, 0, 1.0, 0],
                                                [  0,  0, 0, 0, 1.0]])
        self.process_noise_v_k_minus_1 = np.array([0.01,0.01,0.01, 0.2, 0.2])
        self.Q_k = np.array([[1.0,  0,   0, 0, 0],
                                                [  0,1.0,   0, 0, 0],
                                                [  0,  0, 1.0, 0, 0],
                                                [  0,  0, 0, 1.0, 0],
                                                [  0,  0, 0, 0, 1.0]])
        self.H_k = np.array([[1.0,  0,   0, 0, 0],
                                                [  0,1.0,   0, 0, 0],
                                                [  0,  0, 1.0, 0, 0],
                                                [  0,  0, 0, 1.0, 0],
                                                [  0,  0, 0, 0, 1.0]])
        self.R_k = np.array([[1.0,  0,   0, 0, 0],
                                                [  0,1.0,   0, 0, 0],
                                                [  0,  0, 1.0, 0, 0],
                                                [  0,  0, 0, 1.0, 0],
                                                [  0,  0, 0, 0, 1.0]])
        self.sensor_noise_w_k = np.array([0.07,0.07,0.01, 0.2, 0.2])
        self.z_k =  np.array([0.0,0.0,0.0,0.0,0.0])
        self.state_estimate_k_minus_1 = np.array([0.0,0.0,0.0,0.0,0.0])
        self.control_vector_k_minus_1 = np.array([0.0,0.0])            
        self.P_k_minus_1 = np.array([[0.1,  0,   0, 0, 0],
                                                [  0,0.1,   0, 0, 0],
                                                [  0,  0, 0.1, 0, 0],
                                                [  0,  0, 0, 0.1, 0],
                                                [  0,  0, 0, 0, 0.1]])
    
    def odom_callback(self,msg):
        # print(msg.pose.pose.position.x)
        self.z_k[0] = msg.pose.pose.position.x 
        self.z_k[1] = msg.pose.pose.position.y
        self.z_k[2] = msg.pose.pose.position.z
        self.z_k[3] = msg.twist.twist.linear.x
        self.z_k[4] = msg.twist.twist.angular.z

        # print(msg.pose.pose)
    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z 
    def tf_callback(self,tf):
        a = 1
    def cmd_callback(self, msg):
        self.control_vector_k_minus_1[0] = msg.linear.x
        self.control_vector_k_minus_1[1] = msg.angular.z
    def main_cicle(self):
        optimal_state_estimate_k, covariance_estimate_k = self.ekf(
            self.z_k, # Most recent sensor measurement
            self.state_estimate_k_minus_1, # Our most recent estimate of the state
            self.control_vector_k_minus_1, # Our most recent control input
            self.P_k_minus_1, # Our most recent state covariance matrix
            dt = 0.05) # Time interval
         
        # Get ready for the next timestep by updating the variable values
        self.state_estimate_k_minus_1 = optimal_state_estimate_k
        self.P_k_minus_1 = covariance_estimate_k
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(optimal_state_estimate_k[2] / 2)
        quaternion.w = cos(optimal_state_estimate_k[2] / 2)
        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        transform_stamped_msg.header.frame_id = 'odom'
        transform_stamped_msg.child_frame_id = 'base_link'
        transform_stamped_msg.transform.translation.x = optimal_state_estimate_k[0]
        transform_stamped_msg.transform.translation.y = optimal_state_estimate_k[1]
        transform_stamped_msg.transform.translation.z = 0.0
        transform_stamped_msg.transform.rotation.x = quaternion.x
        transform_stamped_msg.transform.rotation.y = quaternion.y
        transform_stamped_msg.transform.rotation.z = quaternion.z
        transform_stamped_msg.transform.rotation.w = quaternion.w
        self.odom_broadcaster.sendTransform(transform_stamped_msg)
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(optimal_state_estimate_k[2] / 2)
        quaternion.w = cos(optimal_state_estimate_k[2] / 2)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = optimal_state_estimate_k[0]
        odom.pose.pose.position.y = optimal_state_estimate_k[1]
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = optimal_state_estimate_k[3]
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = optimal_state_estimate_k[4]
        self.odom_pub.publish(odom)
        # print(optimal_state_estimate_k)
    def getB(self, yaw, deltat):
        B = np.array([  [np.cos(yaw)*deltat, 0],
                                    [np.sin(yaw)*deltat, 0],
                                    [0, deltat],
                                    [1.0, 0.0],
                                    [0.0, 1.0]])
        return B
    def ekf(self, z_k_observation_vector, state_estimate_k_minus_1, 
        control_vector_k_minus_1, P_k_minus_1, dt):
        state_estimate_k = self.A_k_minus_1 @ (
            state_estimate_k_minus_1) + (
            self.getB(state_estimate_k_minus_1[2],dt)) @ (
            control_vector_k_minus_1) + (
            self.process_noise_v_k_minus_1)
        P_k = self.A_k_minus_1 @ P_k_minus_1 @ self.A_k_minus_1.T + (
            self.Q_k)
        measurement_residual_y_k = z_k_observation_vector - (
            (self.H_k @ state_estimate_k) + (
            self.sensor_noise_w_k))
        # print(measurement_residual_y_k)
        S_k = self.H_k @ P_k @ self.H_k.T + self.R_k
        K_k = P_k @ self.H_k.T @ np.linalg.pinv(S_k)
        # print(K_k)
        state_estimate_k = state_estimate_k + (K_k @ measurement_residual_y_k)
        P_k = P_k - (K_k @ self.H_k @ P_k)
        return state_estimate_k, P_k

def main(args=None):
    rclpy.init(args=args)
    ekf = EKF()
    rclpy.spin(ekf)
    rclpy.shutdown()
    ekf.destroy_node()
    
    

if __name__ == '__main__':
    main()