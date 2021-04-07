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
        self.ir_pub = self.create_publisher(UInt32MultiArray, 'ir_array', 1)
        self.bat_pub = self.create_publisher(Float32, 'battery_voltage', 1)
        self.us_pub = self.create_publisher(Int32MultiArray, 'us_sensors', 1)
        self.subscription_1 = self.create_subscription(Bool,'lift',self.lift_callback,10) #true - up, false - down
        self.line = []
        self.jointstate = JointState()
        self.num_of_wheels = num_of_wheels
        self.jointstate.header.frame_id = 'base_link'
        for i in range(0, self.num_of_wheels):
            self.jointstate.name.append("motor_"+str(i+1))
            self.jointstate.position.append(0)
            self.jointstate.effort.append(0)
            self.jointstate.velocity.append(0)
        self.array_msg = UInt32MultiArray()
        self.us_msg = Int32MultiArray()
        self.voltage = Float32()
        self.previous_cmd_time = time()
        param = self.declare_parameter('usb', '/dev/ttyACM1')
        if self.get_parameter('usb').get_parameter_value().string_value is not None:
            param = self.get_parameter('usb').get_parameter_value().string_value
            print("opencr connected with port: "+param)
        else:
            param = '/dev/ttyACM1'
        self.ser = serial.Serial(param, 115200 ,timeout=1.0)
        self.create_timer(0.5, self.read)

    def lift_callback(self,data):
        if data.data == True:
            s = str(1)+"\n"
            self.ser.write(s.encode())
        else:
            s = str(0)+"\n"
            self.ser.write(s.encode())

    def read(self):
        try:
            b = str(self.ser.readline())
            # print(b)
            b = b.replace('b', '')
            b = b.replace("'",'')
            b = b.replace("n",'')
            b = b.replace("r",'')
            b = b.replace("\\",'')
            b = b.split(',')
            if b[0] == 'light':
                for i in range(1,29):
                    self.array_msg.data.append(int(b[i]))
                self.ir_pub.publish(self.array_msg)
                self.array_msg.data = []
            elif b[0] == 'vol':
                self.voltage.data = float(b[1])
                self.bat_pub.publish(self.voltage)
            elif b[0] == 'us':
                for i in range(1,7):
                    self.us_msg.data.append(int(b[i]))
                self.us_pub.publish(self.us_msg)
                self.us_msg.data = []
        except Exception as e:
            print(e)
def main(args=None):
    rclpy.init(args=args)
    conn = SerialControl(2)
    rclpy.spin(conn)
    conn.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

