#!/usr/bin/env python
import rospy
import sys
import numpy as np

from math import pi, atan2, asin, sqrt, sin, cos
from dynamixel_msgs.msg import MotorStateList
from std_msgs.msg import Float64

# arm dimension
a1 = 0.09 # joint1 to joint2
a2 = 0.07 # joint2 to motor3
H = 0.04 # base to joint1
L = 0.06 # motor3 to TCP

# RRT parameters
epsilon   = 0.04 # Each distance from last node to next node
tolerance = 5e-3 # Distance tolerance to goal 
X_DIM = 0.18
Z_DIM = 0.22

def position2rad(pos):
	pos_rad = [None, None]
	for i in range(0, len(pos)):
		pos_rad[i] = -pi + 2* pi / 1024 * pos[i]
	return pos_rad

class AvoidObstacle(object):
	def __init__(self, x, z):
		self.node_name = rospy.get_name()
		# Motor angles, in rad
		self.arm_pos = None
		self.initial_pos = None
		# Target position
		self.q_goal = np.array([x, z])
		# Publishers and subscribers
		self.pub_pan = rospy.Publisher("/pan_controller/command", Float64, queue_size = 1)
		self.pub_tilt = rospy.Publisher("/tilt_controller/command", Float64, queue_size = 1)
		self.pub_gripper = rospy.Publisher("/gripper_controller/command", Float64, queue_size = 1)
		self.sub_pos = rospy.Subscriber("/motor_states/pan_tilt_port", MotorStateList, self.cb_pos, queue_size = 10)
		
		# Obstacle, a line segment
		self.p1 = np.array([0.0685, 0.1465])
		self.p2 = np.array([0.105, 0.148])
		self.obstacle = np.array([self.p1, self.p2])
		# RRT node
		self.q_size = 1
		self.q_list = None
		self.q_near = None
		self.waypoint = None
		self.waypoint_size = 1
	
	def cb_pos(self, msg):
		self.arm_pos = position2rad([msg.motor_states[0].position, msg.motor_states[1].position])
		self._FK()
		self._rrt()

	# RRT related	
	def _rrt(self):
		while (np.linalg.norm(self.q_goal - self.q_near) > tolerance):
			# Generate q_rand
			self.q_rand = [np.random.random()* 2* X_DIM - X_DIM, 
			       	       np.random.random()* Z_DIM]
			self.q_near, index = self._rrt_near()
			self._rrt_check()
			if(self._rrt_is_hit_obstacle(self.q_list[index], self.q_near) == 0): # Not hit constrain
				self.q_list = np.append(self.q_list, [[0, 0, 0]], axis = 0)
				self.q_list[self.q_size][0:2] = self.q_near
				self.q_list[self.q_size][2] = int(index+1)
				self.q_size = self.q_size + 1
				#print "Add a node in list"
			# end if
			if self.q_size % 100 == 0:
				print "There are ", self.q_size, " nodes in list"
		# end while
		# Reverse tracking q_list
		rospy.loginfo("[%s] Start reverse tracking..." %(self.node_name))
		waypoint_index_list = [self.q_size]
		child_index = int(self.q_size)	
		parent_index = int(self.q_list[self.q_size-1][2])
		while (child_index != 1):
			self.waypoint_size = self.waypoint_size + 1
			waypoint_index_list.append(parent_index)
			temp = parent_index
			parent_index = int(self.q_list[temp-1][2])
			child_index = temp
		# end while
		print "There are ", self.waypoint_size,  "waypoints"
		for i in range(self.waypoint_size, 0, -1):
			x = self.q_list[waypoint_index_list[i-1]-1][0]
			z = self.q_list[waypoint_index_list[i-1]-1][1]
			print "Go to ", x, z
			self._IK(x, z)
		
		self.pub_gripper.publish(Float64(2.0))
		rospy.loginfo("End process")
		rospy.signal_shutdown("End process")

		

	def _rrt_near(self):
		min_dis = 1e6
		index = 0
		L, _ = self.q_list.shape
		
		for i in range(1,L):
			if np.linalg.norm(self.q_rand - self.q_list[i][0:2]) < min_dis:
				min_dis = np.linalg.norm(self.q_rand - self.q_list[i][0:2])
				index = i
			# end if
		# end for
		temp = self.q_rand - self.q_list[index][0:2]
		q_near = self.q_list[index][0:2] + temp / np.linalg.norm(temp) * epsilon
		
		return q_near, index

	def _rrt_check(self):
		if self.q_near[0] > X_DIM:
			self.q_near[0] = X_DIM
		if self.q_near[0] < -X_DIM:
			self.q_near[0] = -X_DIM
		if self.q_near[1] > Z_DIM:
			self.q_near[1] = Z_DIM
		if self.q_near[1] < -Z_DIM:
			self.q_near[1] = -Z_DIM
	
	def _rrt_is_hit_obstacle(self, q_now, q_near):
		A1 = q_near[0] - q_now[0]
		B1 = self.p1[0] - self.p2[0]
		C1 = self.p1[0] - q_now[0]
		A2 = q_near[1] - q_now[1]
		B2 = self.p1[1] - self.p2[1]
		C2 = self.p1[1] - q_now[1]
		t = (B2* C1 - B1* C2)/ (A1* B2 - A2* B1)
		s = (A1* C2 - A2* C1)/(A1* B2 - A2* B1)
		if t > 0 and t < 1 and s > 0 and s < 1:
			return 1
		else:
			return 0 
	# end RRT related
	def _FK(self):
		s1 = sin(self.arm_pos[0])
		c1 = cos(self.arm_pos[0])
		s12 = sin(self.arm_pos[0] + self.arm_pos[1])
		c12 = cos(self.arm_pos[0] + self.arm_pos[1])
		ini_x = a1 * s1 + a2 * s12 + L * c12
		ini_z = a1 * c1 + a2 * c12 - L * s12 + H
		self.initial_pos = [ini_x, ini_z]
		self.q_list = np.array([[ini_x, ini_z, 1]])
		self.q_near = np.array([ini_x, ini_z])
		#rospy.loginfo("[%s] Initial pose: (%s, %s)", %(self.node_name, ini_x, ini_z))
		print ini_x, ini_z
	def _IK(self, x, z):

		
		theta = atan2(z-H, x)
		k = x**2 + (z-H)**2 + a1**2 - a2**2 - L**2
		theta_1_1 = asin(k/(2*a1*sqrt(x**2 + (z-H)**2))) - theta
		theta_1_2 = pi - asin(k/(2*a1*sqrt(x**2 + (z-H)**2))) - theta
		x_bar_1 = x - a1* sin(theta_1_1)
		z_bar_1 = (z-H) - a1 * cos(theta_1_1)
		x_bar_2 = x - a1* sin(theta_1_2)
		z_bar_2 = (z-H) - a1 * cos(theta_1_2)
		theta_2_1 = atan2((a2*x_bar_1 - L* z_bar_1), (a2*z_bar_1 + L*x_bar_1)) - theta_1_1
		theta_2_2 = atan2((a2*x_bar_2 - L* z_bar_2), (a2*z_bar_2 + L*x_bar_2)) - theta_1_2
		ans_1 = [theta_1_1, theta_2_1]
		ans_2 = [theta_1_2, theta_2_2]
		
		ans_1 = self._check_bound(ans_1)
		ans_2 = self._check_bound(ans_2)
		
		# Only one solution
		if ans_1 == None:
			self.nearest = ans_2
		elif ans_2 == None:
			self.nearest = ans_1
		# No solution
		elif ans_1 == None and ans_2 == None:
			self.nearest = None
			rospy.loginfo("Given position not reachable")
			rospy.signal_shutdown("Failed")
		# Two solutions
		# Find nearest solution
		# In L2 distance of joint space
		else:
			dis_1 = (ans_1[0]- self.arm_pos[0])**2 + (ans_1[1] - self.arm_pos[1]) ** 2
			dis_2 = (ans_2[0]- self.arm_pos[0])**2 + (ans_2[1] - self.arm_pos[1]) ** 2
			if dis_1 < dis_2:
				self.nearest = ans_1
			else:
				self.nearest = ans_2
		
		self.nearest[0] = float(format(self.nearest[0], '.3f'))
		self.nearest[1] = float(format(self.nearest[1], '.3f'))
		data_1 = Float64(self.nearest[0])
		data_2 = Float64(self.nearest[1])
		rospy.sleep(0.5)
		self.pub_pan.publish(data_1)
		rospy.sleep(0.5)
		self.pub_tilt.publish(data_2)
		rospy.sleep(0.5)
		rospy.loginfo("[%s] Solution: %s" %(self.node_name, self.nearest))
		
	# joint 1 should be in [-2.9, 3]
	# joint 2 should be in [-2, 2]
	def _check_bound(self, ans):
		# Change to [-pi, pi] branch
		for i in range(0, len(ans)):
			if ans[i] > pi:
				ans[i] = 2* pi - ans[i]
			if ans[i] < -pi:
				ans[i] = 2* pi + ans[i]
		if ans[0] > 3 or ans[0] < -2.9:
			ans = None
		if ans[1] > 2 or ans[1] < -2:
			ans = None
		return ans

if __name__ == "__main__":

	if len(sys.argv) != 3:
		print "Not enough arguments!"
		print "Sample input: rosrun ur_modern_driver dynamixel_arm.py 0.06 0.2"
		sys.exit(0)
	rospy.init_node("arm_RRT", disable_signals = True)
	x = float(sys.argv[1])
	z = float(sys.argv[2])
	if z < 0:
		print "Please give z greater than 0"
		sys.exit(0)
	obj = AvoidObstacle(x , z)
	rospy.spin()

