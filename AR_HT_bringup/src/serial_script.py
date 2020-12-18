#!/usr/bin/env python3  

import rospy
import serial
import struct
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Header
from time import sleep, time
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32MultiArray, Float32, Int32MultiArray
class SerialControl():
	def __init__(self, num_of_wheels):
		self.ser = serial.Serial('/dev/ttyACM1', 57600 ,timeout=1.0)
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
		self.ir_pub = rospy.Publisher('ir_array', UInt32MultiArray, queue_size=1)
		self.bat_pub = rospy.Publisher('battery_voltage', Float32, queue_size=1)
		self.us_pub = rospy.Publisher('us_sensors', Int32MultiArray, queue_size=1)

	def read(self):
		b = str(self.ser.readline())
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
			self.array_msg.data.clear()
		if b[0] == 'vol':
			self.voltage.data = float(b[1])
			self.bat_pub.publish(self.voltage)
		if b[0] == 'us':
			for i in range(1,7):
				self.us_msg.data.append(int(b[i]))
			self.us_pub.publish(self.us_msg)
			self.us_msg.data.clear()
		# print(b)
	
if __name__ == '__main__':
	rospy.init_node('serial_controller')
	controller = SerialControl(4)
	while not rospy.is_shutdown():
		try:
			controller.read()
		except Exception as e:
			print(e)
	
