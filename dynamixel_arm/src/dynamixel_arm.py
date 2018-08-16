#!/usr/bin/env python
import rospy
import sys

from math import pi, atan2, asin, sqrt, sin, cos
from dynamixel_msgs.msg import MotorStateList
from std_msgs.msg import Float64

# arm dimension
a1 = 0.09 # joint1 to joint2
a2 = 0.07 # joint2 to motor3
H = 0.04 # base to joint1
L = 0.06 # motor3 to TCP

def position2rad(pos):
	pos_rad = [None, None]
	for i in range(0, len(pos)):
		pos_rad[i] = -pi + 2* pi / 1024 * pos[i]
	return pos_rad

class IK(object):
	def __init__(self, x, z, gripper_command):
		self.node_name = rospy.get_name()
		# Motor angles, in rad
		self.arm_pos = None
		# Target position
		self.x = x
		self.z = z
		# Gripper command
		# True: pinch up
		# False: lay down
		# None: not motion
		self.gripper_command = gripper_command
		# Publishers and subscribers
		self.pub_pan = rospy.Publisher("/pan_controller/command", Float64, queue_size = 1)
		self.pub_tilt = rospy.Publisher("/tilt_controller/command", Float64, queue_size = 1)
		self.pub_gripper = rospy.Publisher("/gripper_controller/command", Float64, queue_size = 1)
		self.sub_pos = rospy.Subscriber("/motor_states/pan_tilt_port", MotorStateList, self.cb_pos, queue_size = 10)
		
	
	def cb_pos(self, msg):
		self.arm_pos = position2rad([msg.motor_states[0].position, msg.motor_states[1].position])
		for i in range(0, len(msg.motor_states)):
			# Not test yet
			if abs(msg.motor_states[i].load) >= 1:
				data = Float64()
				rospy.sleep(0.5)
				self.pub_pan.publish(data)
				rospy.sleep(0.5)
				self.pub.tilt(data)
				rospy.sleep(0.5)
				self.pub_gripper(data)
				rospy.loginfo("[%s] Emergency stop! Go to home.")
				rospy.signal_shutdown("Enengency stop")
		try:
			# Process inverse kinematic
			self._IK()
			# Publish
			data_1 = Float64(self.nearest[0])
			data_2 = Float64(self.nearest[1])
			rospy.sleep(0.5)
			self.pub_pan.publish(data_1)
			rospy.sleep(0.5)
			self.pub_tilt.publish(data_2)
			if self.gripper_command == "True":
				rospy.sleep(0.5)
				self.pub_gripper.publish(Float64(1.5))
			elif self.gripper_command == "False":
				rospy.sleep(0.5)
				self.pub_gripper.publish(Float64(0))
			
			rospy.loginfo("End process")
			rospy.signal_shutdown("End process")
		except ValueError as e:
			print e
			rospy.loginfo("Given position not reachable")
			rospy.signal_shutdown("Failed")

	def _IK(self):
		x = self.x
		z = self.z
		
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
		
		# Find nearest solution
		# In L2 distance of joint space
		dis_1 = (ans_1[0]- self.arm_pos[0])**2 + (ans_1[1] - self.arm_pos[1]) ** 2
		dis_2 = (ans_2[0]- self.arm_pos[0])**2 + (ans_2[1] - self.arm_pos[1]) ** 2
		if dis_1 < dis_2:
			self.nearest = ans_1
		else:
			self.nearest = ans_2
		
		self.nearest[0] = float(format(self.nearest[0], '.3f'))
		self.nearest[1] = float(format(self.nearest[1], '.3f'))
		rospy.loginfo("[%s] Solution: %s" %(self.node_name, self.nearest))
		
	# joint should be in [-pi, pi]
	def _check_bound(self, ans):
		for i in range(0, len(ans)):
			if ans[i] > pi:
				ans[i] = 2* pi - ans[i]
			if ans[i] < -pi:
				ans[i] = 2* pi + ans[i]
		return ans

if __name__ == "__main__":
	if len(sys.argv) != 4:
		print "Not enough arguments!"
		print "Sample input: rosrun ur_modern_driver dynamixel_arm.py 0.06 0.2 None"
		sys.exit(0)
	rospy.init_node("arm_IK", disable_signals = True)
	x = float(sys.argv[1])
	z = float(sys.argv[2])
	gripper_command = str(sys.argv[3])
	obj = IK(x , z, gripper_command)
	rospy.spin()

