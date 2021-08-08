import rclpy
from rclpy.node import Node
import serial
import struct
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Header
from time import sleep, time
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32MultiArray, Float32, Int32MultiArray, Bool
class SerialControl(Node):
    def __init__(self, num_of_wheels):
        super().__init__('serial_connection')
        self.subscription_1 = self.create_subscription(Bool,'gripper',self.gripper_callback,10) #true - up, false - down
        # param = self.declare_parameter('usb', '/dev/ttyACM2')
        # if self.get_parameter('usb').get_parameter_value().string_value is not None:
        #     param = self.get_parameter('usb').get_parameter_value().string_value
        #     print("opencr connected with port: "+param)
        # else:
        #     param = '/dev/ttyACM2'
        param = '/dev/ttyACM0'
        self.port = serial.Serial(param, 115200 ,timeout=1.0)
        # self.create_timer(0.5, self.write)
        self.flag = 1
    def gripper_callback(self, data):
        if data.data == True:
            start = '1'.encode()
            self.port.write(start)
            print("start")
        else:
            stop = '2'.encode()
            self.port.write(stop)
            print("stop")
    def write(self):
        # b = self.port.readline()
        # print(b)
        print(self.flag)
        start = '1'.encode()
        stop = '2'.encode()
        if(self.flag == 1):
            self.port.write(start)
        else:
            self.port.write(stop)

def main(args=None):
    rclpy.init(args=args)
    conn = SerialControl(2)
    rclpy.spin(conn)
    conn.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

