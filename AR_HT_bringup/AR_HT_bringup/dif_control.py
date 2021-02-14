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

class DifControl():
	def __init__(self, ser):	
		self.motor_r = MotorControl(0, ser)
		self.motor_l = MotorControl(1, ser)
		self.R = 0.115
		self.l = 0.5081
	def set_speed(self, vX, wZ): #meteres per second / radians per second
		v_r = (2*vX - self.l*wZ)/(2*self.R)
		v_l = -1*(2*vX + self.l*wZ)/(2*self.R)
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
		return v_r, v_l
	def init_motors(self, holl, acc, br):
		self.motor_r.holl_impulse(holl)
		self.motor_r.sef_acceleration(acc)
		self.motor_r.set_brake(br)
		self.motor_r.turn_on()
		
		self.motor_l.holl_impulse(holl)
		self.motor_l.sef_acceleration(acc)
		self.motor_l.set_brake(br)
		self.motor_l.turn_on()



def sign(value):
	if value >= 0:
		return 1
	else:
		return -1
def cb(data):
	global v_X_targ, w_Z_targ, control_timeout 
	print(data)
	if abs(data.linear.x) > 0.01 and abs(data.linear.x) < 0.09:
		v_X_targ = sign(data.linear.x)*0.1
	else:	
		v_X_targ = data.linear.x
	if abs(data.angular.z) > 0.01 and abs(data.angular.z) < 0.1:
		w_Z_targ = sign(data.angular.z)*0.1
	else:	
		w_Z_targ = data.angular.z
	control_timeout = time()

def main():
	rclpy.init()
	node = rclpy.create_node('differential_control')
	param = node.declare_parameter('usb', '/dev/ttyUSB0')
	if node.get_parameter('usb').get_parameter_value().string_value is not None:
		param = node.get_parameter('usb').get_parameter_value().string_value
		print(param)
	else:
		param = '/dev/ttyUSB0'
	sub = node.create_subscription(Twist, 'cmd_vel', cb, 10)
	joints_states_pub = node.create_publisher(JointState, "/joint_states", 4)
	ser = serial.Serial(param, timeout=1.0)
	control_timeout = time()
	dif = DifControl(ser)
	dif.init_motors(3, 16, 16)
	v_X_targ = 0
	w_Z_targ = 0
	msg = JointState()
	msg.name.append("right")
	msg.name.append("left")
	msg.velocity.append(0)
	msg.velocity.append(0)
	while rclpy.ok():
		try:
			if(float(time()) - control_timeout < 1.5): 
				print(v_X_targ)
				dif.set_speed(v_X_targ, w_Z_targ)
			else:
				dif.set_speed(0, 0)
			v_r, v_l = dif.get_motors_speed()
			msg.velocity[0] = v_r
			msg.velocity[1] = v_l
			joints_states_pub.publish(msg)
			sleep(0.1)
		except KeyboardInterrupt:
			break
if __name__ == '__main__':
    main()
    