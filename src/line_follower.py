#!/usr/bin/env python
import roslib
import sys
import rospy
from barc.msg import ECU
from como_image_processing.msg import LineData
from numpy import pi
import time
from numpy import pi


# TO BE MOVED TO A CONFIG FILE
heading_ctrl_Kp= 10.0
heading_ctrl_ctrl_max= 90.0
heading_ctrl_ctrl_min= -90.0

lateral_ctrl_Kp= 438.2795
lateral_ctrl_Ki= 15874.8968
lateral_ctrl_Kd= 3.025
lateral_ctrl_i_max= 10.0
lateral_ctrl_i_min= -10.0
lateral_ctrl_ctrl_max= 90.0
lateral_ctrl_ctrl_min= -90.0

# END OF THE CONFIG VARIABLES

class LineDataFetcher:
	def __init__(self):
		self.data_sub = rospy.Subscriber("/line_data", LineData, self.data_callback, queue_size = 1)
		self.line_data = []

	def data_callback(self, linedata):
		self.line_data = linedata
		return

	def get_linedata(self):
		return self.line_data

class PID_ctrl:
	'''
	Generic discrete time PID controller with integrator anti-windup and control output saturation
	'''

	def __init__(self, Kp = 1., Ki = 0., Kd = 0., i_max = 0., i_min = 0., ctrl_max = 1., ctrl_min = 0.):
		self.Kp = Kp
		self.Ki = Ki
		self.Kd = Kd
		self.i_max = i_max
		self.i_min = i_min
		self.ctrl_max = ctrl_max
		self.ctrl_min = ctrl_min
		self.e_curr = 0.
		self.e_prev = 0.
		self.e_sum = 0.
		self.t_curr = 0.
		self.t_prev = 0.

	def apply_pid(self, des_value, current_value, timestamp):
		self.e_curr = des_value - current_value
		print('error', self.e_curr)
		self.t_curr = timestamp
		dt = self.t_curr - self.t_prev
		print('dt', dt)

        # Proportional control
		p_ctrl = self.Kp * self.e_curr
		print('p_ctrl', p_ctrl)

        # Integral control with anti-windup
		i_ctrl = self.e_sum + self.Ki*dt/2.0*(self.e_curr+self.e_prev)
		i_ctrl = min(i_ctrl, self.i_max)
		i_ctrl = max(i_ctrl, self.i_min)
		print('i_ctrl', i_ctrl)

        # Derivative control
		d_ctrl = self.Kd*(self.e_curr-self.e_prev)/dt
		print('d_ctrl', d_ctrl)

        # Control signal calculation
		ctrl = p_ctrl + i_ctrl + d_ctrl

        # Control saturation
		ctrl = min(ctrl, self.ctrl_max)
		ctrl = max(ctrl, self.ctrl_min)

        # update previous values with current values
		self.e_sum = i_ctrl
		self.e_prev = self.e_curr
		self.t_prev = self.t_curr

		return ctrl
        
class LineFollower:
	def __init__(self):
		self.motor = 6.5
		self.heading_ctrl = []
		self.servo = []
		self.angle_radian = []
		self.angle_degree = []
		self.line_pos_x = []
		self.line_pos_y = []

	def import_linedata(self, line_data):
		print(line_data)
		if (line_data == []):
			return
		self.angle_radian = line_data.angle_radian
		self.angle_degree = line_data.angle_degree
		self.line_pos_x = line_data.line_pos_x
		self.line_pos_y = line_data.line_pos_y
		print(self.angle_degree, self.line_pos_x)
    
	def calculate_ctrl(self, heading_ctrl, lateral_ctrl, timestamp):
		if self.angle_degree == []:
			return
		if self.line_pos_x == []:
			return
		print('Calculating ctrl')
		heading_ctrl = heading_ctrl.apply_pid(self.angle_degree, 90.0 , timestamp) # desired is angle of line, and current is always 90
		lateral_ctrl = lateral_ctrl.apply_pid(0.0, self.line_pos_x, timestamp)
		ctrl = 0.5*heading_ctrl + 0.5*lateral_ctrl
		ctrl = ctrl + 90 # add offset to remap ctrl from [-90, 90] to [0, 180]
		print('heading_ctrl', heading_ctrl)
		print('lateral_ctrl', lateral_ctrl)
		self.heading_ctrl = ctrl
		print('ctrl', ctrl)
    
	def set_servo_control(self):
		if self.heading_ctrl == []:
			self.servo = 30.0
		else:
			self.servo = self.heading_ctrl
		
	
	def get_ECU_data(self):
		return self.motor, self.servo

class ECUPublisher:
	def __init__(self):
		self.ECU = ECU(0, 0)
		self.ECU_pub = rospy.Publisher("/ecu", ECU, queue_size = 1)

	def set_ECU(self, motor, servo):
		self.ECU = ECU(motor, pi/180*servo)

	def publish_ECU(self):
		self.ECU_pub.publish(self.ECU)

def main():
	rospy.init_node("line_follower") #initialize ros node
	rate = rospy.Rate(30)
	
	fetcher = LineDataFetcher()
	publisher = ECUPublisher()
	line_follower = LineFollower()
	heading_ctrl = PID_ctrl(Kp = heading_ctrl_Kp, Ki = 0., Kd = 0., i_max = 0., i_min = 0., ctrl_max = heading_ctrl_ctrl_max, ctrl_min = heading_ctrl_ctrl_min)
	lateral_ctrl = PID_ctrl(Kp = lateral_ctrl_Kp, Ki = lateral_ctrl_Ki, Kd = lateral_ctrl_Kd, i_max = lateral_ctrl_i_max, i_min = lateral_ctrl_i_min, ctrl_max = lateral_ctrl_ctrl_max, ctrl_min = lateral_ctrl_ctrl_min)
	timestamp = time.time()
	heading_ctrl.t_prev = timestamp
	heading_ctrl.t_curr = timestamp
	lateral_ctrl.t_prev = timestamp
	lateral_ctrl.t_curr = timestamp

	while not rospy.is_shutdown():
		timestamp = time.time()
		line_data = fetcher.get_linedata()
		line_follower.import_linedata(line_data)
		line_follower.calculate_ctrl(heading_ctrl, lateral_ctrl, timestamp)
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
