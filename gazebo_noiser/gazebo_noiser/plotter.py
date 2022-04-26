import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import random
import math
import time
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import threading
import matplotlib.pyplot as plt
class OdomNoise(Node):
    def __init__(self):
        super().__init__('odom_noise')
        self.sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.sub = self.create_subscription(Odometry, 'odom_noisy', self.odom_noisy_cb, 10)
        self.x = []
        self.y = []
        self.theta = []
        self.x_noisy = []
        self.y_noisy = []
        self.theta_noisy = []
    def odom_cb(self,data):
        self.x.append(data.pose.pose.position.x)
        self.y.append(data.pose.pose.position.y)
        _, _, th = self.euler_from_quaternion(data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        self.theta.append(th)
    def odom_noisy_cb(self,data):
        self.x_noisy.append(data.pose.pose.position.x)
        self.y_noisy.append(data.pose.pose.position.y)
        _, _, th = self.euler_from_quaternion(data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        self.theta_noisy.append(th)
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
    thread = threading.Thread(target=rclpy.spin, args=(noiser, ), daemon=True)
    thread.start()
    while rclpy.ok():
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            print("bye")
            rclpy.shutdown()            
            noiser.destroy_node()
            thread.join()
    fig, ax = plt.subplots()
    x = noiser.x
    y = noiser.y
    x_noise = noiser.x_noisy
    y_noise = noiser.y_noisy
    ax.plot(x, y, label = 'true estimation', linewidth = 1)
    ax.plot(x_noise, y_noise, label = 'noise estimation', linewidth = 1)
    ax.set_xlabel("X")
    ax.set_ylabel("y")
	# fig.set_figwidth(12)
	# fig.set_figheight(12)
    ax.legend()
    plt.grid()
    plt.show()
if __name__ == '__main__':
    main()
