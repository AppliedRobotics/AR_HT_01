import rclpy
from rclpy.node import Node
from AR_HT_navigation.action import ToPoseClient
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
class Controller(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_action_client')
        self.sub_scan_left = self.create_subscription(LaserScan, 'scan_left', self.left_cb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.sub_scan_right = self.create_subscription(LaserScan, 'scan_right', self.right_cb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.sub_scan_front = self.create_subscription(LaserScan, 'scan_front', self.front_cb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    def left_cb(self,data):
        a = data
    def right_cb(self,data):
        a = data
    def front_cb(self,data):
        a = data
    
def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()