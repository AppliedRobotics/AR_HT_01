import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from math import sin, cos, atan2, pi
import numpy as np
import json
import threading
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from time import sleep

class ToPoseClient(Node):
    def __init__(self):
        rclpy.init(args=None)
        super().__init__('navigate_to_pose_action_client_api')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.initialpose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        self.feedback_pub = self.create_publisher(String, "/navigate_feedback", 1)
        self.subscription_9 = self.create_subscription(LaserScan,'scan',self.scan_callback_parking,1)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 4)
        self.parking_flag = False
        self.kx = 3.0
        self.kz = 10.0
        self.move_status = False
        self.feedback = {"state": "stay", 
        "number_of_recoveries": 0,
        "distance_remaining": 0,
        "navigation_time": 0,
        "target": {'x':0, "y":0, "angle":0},
        "current_pose": {'x':0, "y":0, "angle":0}}
        self.target = [0,0,0]
        thread = threading.Thread(target=rclpy.spin, args=(self, ), daemon=True)
        thread.start()
        
                    #states: moving to goal, goal reached, goal cant be reached
    def euler_to_quaternion(self, yaw):
        pitch = 0
        roll = 0
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]
    def set_pose(self,x,y,theta):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = float(x)
        pose_msg.pose.pose.position.y = float(y)
        pose_msg.pose.pose.position.z = float(0)
        qx, qy, qz, qw = self.euler_to_quaternion(theta)
        pose_msg.pose.pose.orientation.w = qw
        pose_msg.pose.pose.orientation.x = qx
        pose_msg.pose.pose.orientation.y = qy
        pose_msg.pose.pose.orientation.z = qz
        # print(pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.w)
        pose_msg.pose.covariance[0] = 0.0001
        pose_msg.pose.covariance[7] = 0.0001
        pose_msg.pose.covariance[35] = 0.0001
        self.initialpose.publish(pose_msg)

    def send_goal(self, x,y,theta):
        self.target = [x,y,theta]
        self.feedback["state"]="moving to goal"
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        qx, qy, qz, qw = self.euler_to_quaternion(theta)
        goal_msg.pose.pose.orientation.w = qw
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        # print(goal_msg.pose.pose.orientation.z, goal_msg.pose.pose.orientation.x, goal_msg.pose.pose.orientation.w, goal_msg.pose.pose.orientation.y)
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        # self.goal_handle = self._send_goal_future.result()    
        # self.result_future = self.goal_handle.get_result_async()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.feedback["state"] = "goal cant be reached"
            return

        self.feedback["state"]="moving to goal"

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # self.status = self.result_future.result().status
        # print(self.status)
        result = future.result().result
        print(result)
        self.get_logger().info('Result: {0}'.format(result))
        if self.feedback["number_of_recoveries"] < 6:
            if self.feedback["distance_remaining"] < 0.4:
                self.feedback["state"] = "goal reached"
            else:
                self.send_goal(self.target[0], self.target[1], self.target[2])
        else:
            self.feedback["state"] = "goal cant be reached"
    
    def feedback_callback(self, feedback_msg):
        # self.status = self.result_future.result().status
        feedback = feedback_msg.feedback
        print(feedback)
        self.feedback["number_of_recoveries"] = feedback.number_of_recoveries
        self.feedback["distance_remaining"] = feedback.distance_remaining
        self.feedback["navigation_time"] = feedback.navigation_time.sec
        self.feedback["target"] = {"x":round(self.target[0], 2), "y":round(self.target[1], 2), "angle":round(self.target[2], 2)}
        x = feedback.current_pose.pose.position.x
        y = feedback.current_pose.pose.position.y
        q = feedback.current_pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        alpha = atan2(siny_cosp, cosy_cosp)+pi/2
        self.feedback["current_pose"] = {"x":round(x,2), "y":round(y,2), "angle":round(alpha,2)}
        msg = String()
        msg.data = json.dumps(self.feedback)
        self.feedback_pub.publish(msg)
        # self.get_logger().info('Received feedback: {0}'.format(feedback))
    
    def get_feedback(self):
        return self.feedback

    def check_constrain(self, err, constr):
        if abs(err) > constr:
            if err > 0:
                return constr
            else:
                return -1*constr
        else:
            return err

    def parking(self):
        self.parking_flag = True
        rate = self.create_rate(2)
        while self.parking_flag == True:
            rate.sleep()
        return self.parking_flag

    def scan_callback_parking(self, data):
        if self.parking_flag is True:
            # print(len(data.ranges))
            z_err = 0
            z_err = self.kz*(data.ranges[353] - data.ranges[383])
            print("ranges: ", data.ranges[343], data.ranges[383])
            print(z_err)
            x_err = self.kx*(data.ranges[363]- 0.12)
            x_err = self.check_constrain(x_err, 0.1)
            z_err = self.check_constrain(z_err, 0.5)
            
            msg = Twist()
            msg.linear.x = x_err
            # msg.linear.y = y_err
            msg.angular.z = z_err
            self.cmd_vel_pub.publish(msg)
            # if(abs(x_err) < 0.06):
                # x_err = 0
            if(abs(x_err) < 0.02  and abs(z_err) < 0.05):
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.angular.z = 0.0
                self.cmd_vel_pub.publish(msg)
                self.parking_flag = False
                print("parked")         
        
    def get_back(self, time):
        msg = Twist()
        msg.linear.x = -0.3     
        self.cmd_vel_pub.publish(msg)
        sleep(2)
        msg.linear.x = 0.0
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    # rclpy.init(args=args)
    action_client = ToPoseClient()
    action_client.parking()
    rclpy.spin()
    # thread = threading.Thread(target=rclpy.spin, args=(action_client, ), daemon=True)
    # thread.start()
    # rate = action_client.create_rate(2)
    # # future = action_client.send_goal(1.0,0.0,0.1)
    # # -3.64, -2.77, 0
    # action_client.set_pose(1.0,2.0,0.0)
    # while rclpy.ok():
    #     try:
    #         action_client.parking()
    #         print(action_client.get_feedback())
    #         rate.sleep()
    #     except KeyboardInterrupt:
    #         rclpy.shutdown()

if __name__ == '__main__':
    main()
