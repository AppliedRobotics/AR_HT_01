import struct
from binascii import hexlify
from codecs import encode
from time import sleep, time
import serial
from math import pi
from AR_HT_bringup.MotorControl import MotorControl
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
import threading
class DifControl(Node):
	def __init__(self):	
		super().__init__('dif_control')
		self.R = 0.115
		self.l = 0.430
		param = self.declare_parameter('usb', '/dev/ttyUSB0')
		if self.get_parameter('usb').get_parameter_value().string_value is not None:
			param = self.get_parameter('usb').get_parameter_value().string_value
			print('connected to:'+param)
		else:
			param = '/dev/ttyUSB0'
		self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
		self.joints_states_pub = self.create_publisher(JointState, "/joint_states", 4)
		self.ser = serial.Serial(param, timeout=1.0)
		self.motor_r = MotorControl(0, self.ser)
		self.motor_l = MotorControl(1, self.ser)
		self.control_timeout = time()
		self.init_motors(4, 16, 16)
		self.v_X_targ = 0
		self.w_Z_targ = 0
		self.msg = JointState()
		self.msg.name.append("right")
		self.msg.name.append("left")
		self.msg.velocity.append(0)
		self.msg.velocity.append(0)
		self.create_timer(0.05, self.timer_callback)
	def cmd_cb(self, data):
		if abs(data.linear.x) > 0.01 and abs(data.linear.x) < 0.09:
			self.v_X_targ = self.sign(data.linear.x)*0.1/2
		else:	
			self.v_X_targ = data.linear.x/2
		if abs(data.angular.z) > 0.01 and abs(data.angular.z) < 0.1:
			self.w_Z_targ = self.sign(data.angular.z)*0.1/2
		else:	
			self.w_Z_targ = data.angular.z/2
		self.control_timeout = time()
	def timer_callback(self):
		v_r, v_l = self.get_motors_speed()
		if(float(time()) - self.control_timeout < 1.5): 
			self.set_speed(self.v_X_targ, self.w_Z_targ, v_r, v_l)
			if self.v_X_targ == 0.0 and self.w_Z_targ == 0.0:
				v_l = 0.0
				v_r = 0.0
		else:
			v_r = 0.0
			v_l = 0.0
			self.set_speed(0, 0, v_r, v_l)
		self.msg.velocity[0] = v_r
		self.msg.velocity[1] = v_l
		self.joints_states_pub.publish(self.msg)
	def set_speed(self, vX, wZ, r_v_r, r_v_l): #meteres per second / radians per second
		v_r = (2*vX - self.l*wZ)/(2*self.R)
		v_l = -1*(2*vX + self.l*wZ)/(2*self.R)
		if wZ < 0.32 and abs(vX) < 0.01 and wZ > 0:
			v_r = -0.32
			v_l = -0.32
		if wZ > -0.32 and abs(vX) < 0.01 and wZ < 0:
			v_r = 0.32
			v_l = 0.32
		# print(err_l, err_r)
		# print(v_r, r_v_r, v_l, r_v_l)
		# print("v_r:"+str(v_r)+" v_l:"+str(v_l))
		self.motor_r.goal_velocity(v_r)
		self.motor_l.goal_velocity(v_l)

	def get_motors_speed(self):
		self.motor_r.get_status()
		self.motor_l.get_status()
		if self.motor_r.direction_of_turn == '0':
			v_r = -self.motor_r.motor_speed 
		else:
			v_r = self.motor_r.motor_speed 
		if self.motor_l.direction_of_turn == '1':
			v_l = -self.motor_l.motor_speed 
		else:
			v_l = self.motor_l.motor_speed
		return v_r*2, v_l*2
	def init_motors(self, holl, acc, br):
		self.motor_r.holl_impulse(holl)
		self.motor_r.sef_acceleration(acc)
		self.motor_r.set_brake(br)
		self.motor_r.turn_on()
		
		self.motor_l.holl_impulse(holl)
		self.motor_l.sef_acceleration(acc)
		self.motor_l.set_brake(br)
		self.motor_l.turn_on()
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
    