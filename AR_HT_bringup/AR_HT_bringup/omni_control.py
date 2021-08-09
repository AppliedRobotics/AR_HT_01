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
		self.R = 0.0771
		L1 = 0.5172
		L2 = 0.49
		self.wheel_sep = (L1+L2)/2
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
		self.msg.name.append("left_front")
		self.msg.name.append("right_front")
		self.msg.name.append("left_back")
		self.msg.name.append("right_back")
		self.msg.velocity.append(0)
		self.msg.velocity.append(0)
		self.msg.velocity.append(0)
		self.msg.velocity.append(0)
		self.create_timer(0.05, self.timer_callback)
	def cmd_cb(self, data):
		self.v_X_targ = data.linear.x
		self.v_Y_targ = data.linear.y
		# if abs(data.angular.z) < 0.1:
			# self.w_Z_targ = self.sign(data.angular.z)*0.1
		# else:
		self.w_Z_targ = data.angular.z
		self.control_timeout = time()
	def timer_callback(self):
		v_lf, v_rf, v_lb, v_rb = self.get_motors_speed()
		# print(round(-v_lf,3), round(v_rf,3), round(-v_lb,3), round(v_rb,3))
		if(float(time()) - self.control_timeout < 1.5): 
			self.set_speed(self.v_X_targ, self.v_Y_targ, self.w_Z_targ)
		else:
			self.set_speed(0, 0, 0)
		voltage = Float64()
		voltage.data = self.motors.get_voltage()
		self.voltage_pub.publish(voltage)
		self.msg.velocity[0] = -v_lf
		self.msg.velocity[1] = v_rf
		self.msg.velocity[2] = -v_lb
		self.msg.velocity[3] = v_rb
		self.joints_states_pub.publish(self.msg)
	def set_speed(self, vX, vY, wZ): #meteres per second / radians per second
		vX = -1*vX
		v_lf = (1 / self.R * (vX - vY - self.wheel_sep * (wZ) ) * (-1))/self.k
		v_rf = (1 / self.R * (vX + vY + self.wheel_sep * (wZ) ) * (1))/self.k
		v_lb = (1 / self.R * (- vX - vY + self.wheel_sep * (wZ) ) * (1))/self.k
		v_rb = (1 / self.R * (- vX + vY - self.wheel_sep * (wZ) ) * (-1))/self.k
		
		self.motors.goal_velocity('lf', v_lf)
		self.motors.goal_velocity('rf', v_rf)
		self.motors.goal_velocity('lb', v_lb)
		self.motors.goal_velocity('rb', v_rb)

	def get_motors_speed(self):
		v_lf = self.motors.get_speed('lf')
		v_rf = self.motors.get_speed('rf')
		v_lb = self.motors.get_speed('lb')
		v_rb = self.motors.get_speed('rb')
		return v_lf, v_rf, v_lb, v_rb
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
    