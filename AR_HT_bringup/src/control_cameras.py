#!/usr/bin/env python3  
import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from time import sleep

class CamerasController():
	def __init__(self):
		self.angles = Vector3() #x for lower motor y for upper motor in radians
		#for lower motor from 2pi to 0 radian 
		#for high motor from 2.61 to 4.72 radian
		self.speed = Vector3() #x for lower motor y for upper motor in radians per second
		self.acceleration = Vector3() #[1-253] default 20
		self.angles_pub = rospy.Publisher("target_angles", Vector3, queue_size=1)
		self.speed_pub = rospy.Publisher("target_velocity", Vector3, queue_size=1)
		self.acc_pub = rospy.Publisher("target_acceleration", Vector3, queue_size=1)
		self.joint_state_sub = rospy.Subscriber("joint_state", JointState, self.js_cb)

	def js_cb(self,data):
		a = 1
	def send_angles(self, pose_1, pose_2):
		self.angles.x = pose_1
		self.angles.y = pose_2
		self.angles_pub.publish(self.angles)
	def send_speed(self, speed_1, speed_2):
		self.speed.x = speed_1
		self.speed.y = speed_2
		self.speed_pub.publish(self.speed)
	def send_acc(self, acc_1, acc_2):
		self.acceleration.x = acc_1
		self.acceleration.y = acc_2
		self.acc_pub.publish(self.acceleration)

if __name__ == '__main__':
	rospy.init_node('cameras_controller')
	controller = CamerasController()
	while not rospy.is_shutdown():
		try:
			controller.send_acc(1,1)
			controller.send_speed(12,12)
			controller.send_angles(4,2.7)
			sleep(0.1)
		except KeyboardInterrupt:
			print("something goes wrong")
