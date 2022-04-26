import struct
from binascii import hexlify
from codecs import encode
from time import sleep, time
from math import pi
from AR_HT_bringup.MotorControl_odrive import MotorControl
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
import threading
class DifControl(Node):
	def __init__(self):	
		super().__init__('dif_control')
		self.R = 0.115
		self.l = 0.430
		self.motors = MotorControl()
		self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 2)
		self.joints_states_pub = self.create_publisher(JointState, "/joint_states", 4)
		self.voltage_pub = self.create_publisher(Float64, "/voltage", 4)
		self.control_timeout = time()
		self.v_X_targ = 0
		self.v_Y_targ = 0
		self.k = 0.39
		self.w_Z_targ = 0
		self.msg = JointState()
		self.msg.name.append("left")
		self.msg.name.append("right")
		self.msg.velocity.append(0)
		self.msg.velocity.append(0)
		self.create_timer(0.05, self.timer_callback)
	def cmd_cb(self, data):
		self.v_X_targ = data.linear.x
		self.v_Y_targ = data.linear.y
		self.w_Z_targ = data.angular.z
		self.control_timeout = time()
	def timer_callback(self):
		v_l, v_r = self.get_motors_speed()
		if(float(time()) - self.control_timeout < 1.5): 
			self.set_speed(self.v_X_targ, self.v_Y_targ, self.w_Z_targ)
		else:
			self.set_speed(0, 0, 0)
		voltage = Float64()
		voltage.data = self.motors.get_voltage()
		self.voltage_pub.publish(voltage)
		self.msg.velocity[0] = v_l
		self.msg.velocity[1] = v_r
		self.joints_states_pub.publish(self.msg)
	def set_speed(self, vX, vY, wZ): #meteres per second / radians per second
		wZ = -1*wZ
		v_r = ((2*vX - self.l*wZ)/(2*self.R))/(0.39)
		v_l = (-1*(2*vX + self.l*wZ)/(2*self.R))/(0.39)
		
		self.motors.goal_velocity('l', v_l)
		self.motors.goal_velocity('r', v_r)
		
	def get_motors_speed(self):
		v_l = -1*self.motors.get_speed('l')*0.39
		v_r = self.motors.get_speed('r')*0.39
		return v_l, v_r
	def sign(self, value):
		if value >= 0:
			return 1
		else:
			return -1


def main():
	rclpy.init()
	dif = DifControl()
	rclpy.spin(dif)
	rclpy.shutdown()
if __name__ == '__main__':
    main()
    