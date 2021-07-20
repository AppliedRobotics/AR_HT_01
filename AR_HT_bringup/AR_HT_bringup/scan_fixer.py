from math import pi, sin, cos
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

class ScanFixer(Node):
    def __init__(self):
        super().__init__('scan_fixer')
        self.sub_1 = self.create_subscription(LaserScan, 'scan_front_wrong', self.sc_cb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.scan_pub = self.create_publisher(LaserScan, "scan_front", 10)
        print('scan fixer initialized')
    def sc_cb(self,data):
        data.header.stamp = self.get_clock().now().to_msg()
        i = 0
        for range_ in data.ranges:
            if(i >= 150 and i <= 180):
                data.ranges[i] = 0
                data.intensities[i] = 0
            i+=1
        self.scan_pub.publish(data)
def main():
    rclpy.init()
    fixer = ScanFixer()
    rclpy.spin(fixer)
    rclpy.shutdown()


if __name__ == '__main__':
    main()