import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import threading
import matplotlib.pyplot as plt
import pandas as pd
from time import time
class Bag_to_file(Node):
	def __init__(self):
		super().__init__('odom_noise')
		print("file name pls ->")
		self.filename = input()
		print("ready for writing jonts bag in "+self.filename+".csv file")
		self.sub = self.create_subscription(JointState, 'joint_states', self.js_cb, 10)   
		self.sub_cmd = self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)   
		
		#self.sub_odom = self.create_subscription(Odometry, 'odom_clean', self.odom_cb, 10)   
		self.js_data = {'Time':[],
						'Left_motor':[],
						'Right_motor':[]}
		self.first_msg = True
		self.odom_data = {'Time':[],
						'linear':[],
						'angular':[]}
		self.start_measure = False
		self.t_old = 0
		self.t_ = 0
		self.left_vel = 0
		self.right_vel = 0
		self.create_timer(0.05, self.timer_callback)
		# self.f = open('joint_states.txt', 'w')


	def timer_callback(self):
		if(self.start_measure == True):
			if(self.first_msg == True):
				print("begin writing")
				self.first_msg = False
				self.t_old = time()
				return
		delta_t = time() - self.t_old
		self.t_ += delta_t
		self.js_data['Time'].append(self.t_)
		self.js_data['Left_motor'].append(self.left_vel)
		self.js_data['Right_motor'].append(self.right_vel)
		self.t_old = time()


	def cmd_cb(self, data):
		if(abs(data.linear.x) > 0 or abs(data.angular.z) > 0):
			self.start_measure = True

	def odom_cb(self,data):
		if(self.first_msg == True):
			print("begin writing")
			self.first_msg = False
		self.odom_data['Time'].append(data.header.stamp.sec + data.header.stamp.nanosec*(10**(-9)))
		self.odom_data['linear'].append(data)

	def js_cb(self,data):
		# self.f.write(str(data.velocity[0])+' '+str(data.velocity[1]) + '\n')
		# if(self.start_measure == True):
			# if(self.first_msg == True):
				# print("begin writing")
				# self.first_msg = False
				# self.t_old = time()
				# return
				# t = data.header.stamp.sec + data.header.stamp.nanosec*(10**(-9))
				# self.t_old = t
			# delta_t = time() - self.t_old
			# if(delta_t > 0.065):
			# 	self.t_ += delta_t
			# 	self.js_data['Time'].append(self.t_)
			# 	self.js_data['Left_motor'].append(data.velocity[0])
			# 	self.js_data['Right_motor'].append(data.velocity[1])
			# 	self.t_old = time()
		self.left_vel = data.velocity[0]
		self.right_vel = data.velocity[1]


	def js_to_file(self, data):
		df = pd.DataFrame(data)
		df.to_csv('csv/'+self.filename+'.csv')


def main():
	rclpy.init()
	btf = Bag_to_file()
	while rclpy.ok():
		try:
			rclpy.spin(btf)	
		except KeyboardInterrupt:
			print("bye")
			btf.js_to_file(btf.js_data)
			rclpy.shutdown()            
			btf.destroy_node()
			
	
plt.show()
if __name__ == '__main__':
    main()
