#!/usr/bin/env python
"""
Subscriber ROS node which processes data from pose topics and uses
them to move the robot.
"""

# rospy - ROS Python API
import rospy

# baxter_interface - Baxter Python API
import baxter_interface

# import argparse
# import sys

from std_msgs.msg import (
	Header,
	Float64,
	String,
)

from robotpose import RobotPose
import presets
import fun
import iksolver

def create_subs():

	# rospy.Subscriber('LH_pos_x', Float64, call_LH_pos_x)
	# rospy.Subscriber('LH_pos_y', Float64, call_LH_pos_y)
	# rospy.Subscriber('LH_pos_z', Float64, call_LH_pos_z)

	rospy.Subscriber('RH_pos_x', Float64, call_RH_pos_x)
	rospy.Subscriber('RH_pos_y', Float64, call_RH_pos_y)
	rospy.Subscriber('RH_pos_z', Float64, call_RH_pos_z)

	rospy.Subscriber('reset_pose', String, call_reset_pose)
	rospy.Subscriber('head_pan', Float64, call_head_pan)

	if not presets.FIXED_ORIENTATION:
		rospy.Subscriber('RH_roll', Float64, call_RH_roll)
		rospy.Subscriber('RH_pitch', Float64, call_RH_pitch)
		rospy.Subscriber('RH_yaw', Float64, call_RH_yaw)

		# rospy.Subscriber('LH_roll', Float64, call_LH_roll)
		# rospy.Subscriber('LH_pitch', Float64, call_LH_pitch)
		# rospy.Subscriber('LH_yaw', Float64, call_LH_yaw)

	if presets.LEFT_GRIPPER_CONNECTED:
		rospy.Subscriber('left_grip', Float64, call_left_grip)
	if presets.RIGHT_GRIPPER_CONNECTED:
		rospy.Subscriber('right_grip', Float64, call_right_grip)

# Callback functions that map Oculus pose to robot pose.
# Oculus coordinate system is different from Baxter's
def call_LH_pos_x(data):
	global goalPose
	goalPose.LH_pos_y = -data.data * presets.USER_MAPPING_MULTIPLIER
	#rospy.loginfo("LH pos y : %f", goalPose.LH_pos_y)

def call_LH_pos_y(data):
	global goalPose
	goalPose.LH_pos_z = (data.data + presets.OCULUS_ORIGIN_Y_ADJUSTMENT) * presets.USER_MAPPING_MULTIPLIER
	#rospy.loginfo("LH pos z : %f", goalPose.LH_pos_z)

def call_LH_pos_z(data):
	global goalPose
	goalPose.LH_pos_x = -data.data  * presets.USER_MAPPING_MULTIPLIER
	rospy.loginfo("LH pos x : %f", goalPose.LH_pos_x)

def call_LH_ori_w(data):
	global goalPose
	goalPose.LH_ori_w = data.data
	#rospy.loginfo("LH ori w : %f", goalPose.LH_ori_w)

def call_LH_ori_x(data):
	global goalPose
	goalPose.LH_ori_x = data.data
	#rospy.loginfo("LH ori x : %f", goalPose.LH_ori_x)

def call_LH_ori_y(data):
	global goalPose
	goalPose.LH_ori_y = data.data
	#rospy.loginfo("LH ori y : %f", goalPose.LH_ori_y)

def call_LH_ori_z(data):
	global goalPose
	goalPose.LH_ori_z = data.data
	#rospy.loginfo("LH ori z : %f", goalPose.LH_ori_z)

def call_RH_pos_x(data):
	global goalPose
	goalPose.RH_pos_y = -data.data * presets.USER_MAPPING_MULTIPLIER
	#rospy.loginfo("RH pos y : %f", goalPose.RH_pos_y)

def call_RH_pos_y(data):
	global goalPose
	goalPose.RH_pos_z = (data.data + presets.OCULUS_ORIGIN_Y_ADJUSTMENT) * presets.USER_MAPPING_MULTIPLIER
	#rospy.loginfo("RH pos z : %f", goalPose.RH_pos_z)

def call_RH_pos_z(data):
	global goalPose
	goalPose.RH_pos_x = -data.data * presets.USER_MAPPING_MULTIPLIER
	rospy.loginfo("RH pos x : %f", goalPose.RH_pos_x)

def call_RH_ori_w(data):
	global goalPose
	goalPose.RH_ori_w = data.data
	#rospy.loginfo("RH ori w : %f", goalPose.RH_ori_w)

def call_RH_ori_x(data):
	global goalPose
	goalPose.RH_ori_x = data.data
	#rospy.loginfo("RH ori x : %f", goalPose.RH_ori_x)

def call_RH_ori_y(data):
	global goalPose
	goalPose.RH_ori_y = data.data
	#rospy.loginfo("RH ori y : %f", goalPose.RH_ori_y)

def call_RH_ori_z(data):
	global goalPose
	goalPose.RH_ori_z = data.data
	#rospy.loginfo("RH ori z : %f", goalPose.RH_ori_z)

def call_RH_roll(data):
	goalPose.RH_yaw = -data.data
	# rospy.loginfo("RH Yaw is %f", goalPose.RH_yaw*57.2958)

def call_RH_pitch(data):
	goalPose.RH_pitch = -data.data + 90*0.0174533
	rospy.loginfo("RH Pitch is %f", goalPose.RH_pitch*57.2958)

def call_RH_yaw(data):
	goalPose.RH_roll = -data.data
	rospy.loginfo("RH Roll is %f", goalPose.RH_roll*57.2958)

def reset_pose():
	angles = {'right_s0': -0.7854, 'right_s1': 0, 'right_e0': 0, 'right_e1': 2*0.7854, 
	 			'right_w0': -2*0.7854, 'right_w1': -2*0.7854, 'right_w2': 0}
	right_limb.set_joint_position_speed(1)
	right_limb.move_to_joint_positions(angles)
	right_limb.set_joint_position_speed(presets.JOINT_SPEED)

	# TODO LEFT HAND
	updateCurrentPose()

	#Human-like reset pose
def call_reset_pose(data):
	rospy.loginfo(data.data)
	reset_pose()

def call_left_grip(data):
	if left_gripper.type() != 'electric':
		rospy.logwarn("%s not capable of command", left_gripper)
		return
	grip = 100 - data.data*100 # rescale Oculus Touch gripping
	left_gripper.command_position(grip)

def call_right_grip(data):
	print("Gripping right hand...")
	if right_gripper.type() != 'electric':
		rospy.logwarn("%s not capable of command", right_gripper)
		return
	grip = 100 - data.data*100 # rescale Oculus Touch gripping
	right_gripper.command_position(grip)

def call_head_pan(data):
	global head
	pan = data.data
	if (presets.ROBOT_HEAD_PAN_MIN <= pan <= presets.ROBOT_HEAD_PAN_MAX):
		head.set_pan(pan, timeout=5.0)


def goalReached():
	RH_pos_x_GoalReached = (abs(right_limb.endpoint_pose()["position"].x - goalPose.RH_pos_x) <= presets.PRECISION_TOLERANCE)
	RH_pos_y_GoalReached = (abs(right_limb.endpoint_pose()["position"].y - goalPose.RH_pos_y) <= presets.PRECISION_TOLERANCE)
	RH_pos_z_GoalReached = (abs(right_limb.endpoint_pose()["position"].z - goalPose.RH_pos_z) <= presets.PRECISION_TOLERANCE)

	RH_ori_w_GoalReached = (abs(currentPose.RH_ori_w - goalPose.RH_ori_w) <= presets.PRECISION_TOLERANCE/2)
	RH_ori_x_GoalReached = (abs(currentPose.RH_ori_x - goalPose.RH_ori_x) <= presets.PRECISION_TOLERANCE/2)
	RH_ori_y_GoalReached = (abs(currentPose.RH_ori_y - goalPose.RH_ori_y) <= presets.PRECISION_TOLERANCE/2)
	RH_ori_z_GoalReached = (abs(currentPose.RH_ori_z - goalPose.RH_ori_z) <= presets.PRECISION_TOLERANCE/2)

	# RH_roll_reached = (abs(currentPose.RH_roll - goalPose.RH_roll) <= PRECISION_TOLERANCE/2)
	# RH_pitch_reached = (abs(currentPose.RH_roll - goalPose.RH_roll) <= PRECISION_TOLERANCE/2)
	# RH_yaw_reached = (abs(currentPose.RH_roll - goalPose.RH_roll) <= PRECISION_TOLERANCE/2)

	# TODO left hand
	# LH_pos_x_GoalReached = (abs(left_limb.endpoint_pose()["position"].x - goalPose.LH_pos_x) <= PRECISION_TOLERANCE)
	# LH_pos_y_GoalReached = (abs(left_limb.endpoint_pose()["position"].y - goalPose.LH_pos_y) <= PRECISION_TOLERANCE)
	# LH_pos_z_GoalReached = (abs(left_limb.endpoint_pose()["position"].z - goalPose.LH_pos_z) <= PRECISION_TOLERANCE)
	#if (LH_pos_x_GoalReached & LH_pos_y_GoalReached & LH_pos_z_GoalReached & RH_pos_x_GoalReached & RH_pos_y_GoalReached & RH_pos_z_GoalReached &
	#		[orientations]):

	if (RH_pos_x_GoalReached & RH_pos_y_GoalReached & RH_pos_z_GoalReached &
			RH_ori_w_GoalReached & RH_ori_x_GoalReached & RH_ori_y_GoalReached & RH_ori_z_GoalReached):
		return True
	else:
		return False

def updateCurrentPose():
	global currentPose
	global right_limb
	global left_limb

	currentPose.RH_pos_x = right_limb.endpoint_pose()["position"].x
	currentPose.RH_pos_y = right_limb.endpoint_pose()["position"].y
	currentPose.RH_pos_z = right_limb.endpoint_pose()["position"].z
	currentPose.RH_ori_w = right_limb.endpoint_pose()["orientation"].w
	currentPose.RH_ori_x = right_limb.endpoint_pose()["orientation"].x
	currentPose.RH_ori_y = right_limb.endpoint_pose()["orientation"].y
	currentPose.RH_ori_z = right_limb.endpoint_pose()["orientation"].z

	currentPose.LH_pos_x = left_limb.endpoint_pose()["position"].x
	currentPose.LH_pos_y = left_limb.endpoint_pose()["position"].y
	currentPose.LH_pos_z = left_limb.endpoint_pose()["position"].z
	currentPose.LH_ori_w = left_limb.endpoint_pose()["orientation"].w
	currentPose.LH_ori_x = left_limb.endpoint_pose()["orientation"].x
	currentPose.LH_ori_y = left_limb.endpoint_pose()["orientation"].y
	currentPose.LH_ori_z = left_limb.endpoint_pose()["orientation"].z

def add_roll(current_joint_angles):
	angles=current_joint_angles
	if goalPose.RH_yaw <= presets.WRIST_ROLL_MIN:
		angles['right_w2'] = presets.WRIST_ROLL_MIN
	elif goalPose.RH_yaw >= presets.WRIST_ROLL_MAX:
		angles['right_w2'] = presets.WRIST_ROLL_MAX
	else: angles['right_w2'] = goalPose.RH_yaw
	return angles

if __name__ == '__main__':
	# Initialize the ROS node and enable robot
	rospy.init_node("Baxter_Move_Subscribers")
	rs = baxter_interface.RobotEnable()
	rs.enable()

	# Create an instance of baxter_interface's Limb, Gripper and head classes
	right_limb = baxter_interface.Limb("right")
	left_limb = baxter_interface.Limb("left")
	right_gripper = baxter_interface.Gripper('right')
	left_gripper = baxter_interface.Gripper('left')
	head = baxter_interface.Head()

	# Set limb and gripper parameters
	right_limb.set_joint_position_speed(presets.JOINT_SPEED)
	# left_limb.set_joint_position_speed(presets.JOINT_SPEED)

	if presets.RIGHT_GRIPPER_CONNECTED:
		right_gripper.set_velocity(presets.GRIP_VELOCITY)
		right_gripper.calibrate()

	if presets.LEFT_GRIPPER_CONNECTED:
		left_gripper.set_velocity(presets.GRIP_VELOCITY)
		left_gripper.calibrate()

	# Initialise current and goal pose of the robot
	goalPose = RobotPose()
	currentPose = RobotPose()
	updateCurrentPose()	
	goalPose = currentPose

	# Move to starting pose
	reset_pose()

	# Initialise subscribers
	create_subs()

	# Set up head
	head.set_pan(0)
	presets.display_image()	
	head.command_nod()

	while not rospy.is_shutdown():
		while not goalReached():
			right_limb.set_joint_positions(iksolver.ik_solve('right', goalPose))	
			#left_limb.set_joint_positions(ik_solve('left', goalPose))
			updateCurrentPose()

			if (presets.FIXED_ORIENTATION):
				presets.fixOrientation()
	# quit
	quit()