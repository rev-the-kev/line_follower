#!/usr/bin/env python
import roslib
import sys
import rospy
from barc.msg import ECU
from como_image_processing.msg import LineData
from numpy import pi

class LineDataFetcher:
	def __init__(self):
		self.data_sub = rospy.Subscriber("/line_data", LineData, self.data_callback, queue_size = 1)
		self.line_data = []

	def data_callback(self, linedata):
		self.line_data = linedata
		return

	def get_linedata(self):
		return self.line_data

class LineFollower:
	def __init__(self):
		self.motor = 6.5
		self.servo = []
		self.angle_radian = []
		self.angle_degree = []
		self.line_pos_x = []
		self.line_pos_y = []

	def import_linedata(self, line_data):
		if (line_data == []):
			return
		self.angle_radian = line_data.angle_radian
		self.angle_degree = line_data.angle_degree
		self.line_pos_x = line_data.line_pos_x
		self.line_pos_y = line_data.line_pos_y

	def set_servo_control(self):
		self.servo = 30.0
	
	def get_ECU_data(self):
		return self.motor, self.servo

class ECUPublisher:
	def __init__(self):
		self.ECU = ECU(0, 0)
		self.ECU_pub = rospy.Publisher("/ecu", ECU, queue_size = 1)

	def set_ECU(self, motor, servo):
		self.ECU = ECU(motor, servo)

	def publish_ECU(self):
		self.ECU_pub.publish(self.ECU)

def main():
	rospy.init_node("line_follower") #initialize ros node
	rate = rospy.Rate(30)
	
	fetcher = LineDataFetcher()
	publisher = ECUPublisher()
	line_follower = LineFollower()
	
	while not rospy.is_shutdown():
		line_data = fetcher.get_linedata()
		line_follower.import_linedata(line_data)
		line_follower.set_servo_control()

		motor, servo = line_follower.get_ECU_data()
		publisher.set_ECU(motor, servo)
		publisher.publish_ECU()	
	
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.logfatal("ROS Interrupt. Shutting down line_follower node")
		pass
