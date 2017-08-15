#!/usr/bin/env python
"""
Create ROS subscribers and define callback functions which process data from pose topics 
and use them to move the robot.
"""
import rospy
from std_msgs.msg import (
	Header,
	Float64,
	String,
)
import robot
import presets

RIGHT_ANGLE_RAD = 1.5708

# ROS subscribers reading Oculus position, orientation and button presses
def create_subs():

	rospy.Subscriber('LH_pos_x', Float64, call_LH_pos_x)
	rospy.Subscriber('LH_pos_y', Float64, call_LH_pos_y)
	rospy.Subscriber('LH_pos_z', Float64, call_LH_pos_z)

	rospy.Subscriber('RH_pos_x', Float64, call_RH_pos_x)
	rospy.Subscriber('RH_pos_y', Float64, call_RH_pos_y)
	rospy.Subscriber('RH_pos_z', Float64, call_RH_pos_z)

	rospy.Subscriber('reset_pose', String, call_reset_pose)
	rospy.Subscriber('head_pan', Float64, call_head_pan)

	rospy.Subscriber('RH_mode_roll', Float64, call_RH_mode_roll)
	rospy.Subscriber('LH_mode_roll', Float64, call_LH_mode_roll)

	rospy.Subscriber('RH_roll', Float64, call_RH_roll)
	rospy.Subscriber('RH_pitch', Float64, call_RH_pitch)
	rospy.Subscriber('RH_yaw', Float64, call_RH_yaw)

	rospy.Subscriber('LH_roll', Float64, call_LH_roll)
	rospy.Subscriber('LH_pitch', Float64, call_LH_pitch)
	rospy.Subscriber('LH_yaw', Float64, call_LH_yaw)

	if presets.LEFT_GRIPPER_CONNECTED:
		rospy.Subscriber('left_grip', Float64, call_left_grip)
	if presets.RIGHT_GRIPPER_CONNECTED:
		rospy.Subscriber('right_grip', Float64, call_right_grip)


# Callback functions related to button presses on the Touch controller

def call_RH_mode_roll(data):
	if data.data == 1.0:
		robot.RH_mode_roll = True
	else:
		robot.RH_mode_roll = False

def call_LH_mode_roll(data):
	if data.data == 1.0:
		robot.LH_mode_roll = True
	else:
		robot.LH_mode_roll = False

def call_reset_pose(data):
	rospy.loginfo(data.data) # prints ("Resetting robot pose")
	robot.reset_pose()

def call_left_grip(data):
	if robot.left_gripper.type() != 'electric':
		rospy.logwarn("%s not capable of command", robot.left_gripper)
		return
	grip = 100 - data.data*100 # maps Oculus Touch gripping
	robot.left_gripper.command_position(grip)

def call_right_grip(data):
	if robot.right_gripper.type() != 'electric':
		rospy.logwarn("%s not capable of command", robot.right_gripper)
		return
	grip = 100 - data.data*100 # maps Oculus Touch gripping
	robot.right_gripper.command_position(grip)

def call_head_pan(data):
	pan = data.data
	if (presets.ROBOT_HEAD_PAN_MIN <= pan <= presets.ROBOT_HEAD_PAN_MAX):
		robot.head.set_pan(pan, timeout=5.0)


# Callback functions that map Oculus pose to robot pose.
# Oculus coordinate system is different from Baxter's

def call_LH_pos_x(data):
	robot.goalPose.LH_pos_y = -data.data * presets.USER_MAPPING_MULTIPLIER

def call_LH_pos_y(data):
	robot.goalPose.LH_pos_z = (data.data + presets.OCULUS_ORIGIN_Y_ADJUSTMENT) * presets.USER_MAPPING_MULTIPLIER

def call_LH_pos_z(data):
	robot.goalPose.LH_pos_x = -data.data  * presets.USER_MAPPING_MULTIPLIER


def call_LH_roll(data):
	robot.goalPose.LH_yaw = -data.data

def call_LH_pitch(data):
	robot.goalPose.LH_pitch = -data.data + RIGHT_ANGLE_RAD

def call_LH_yaw(data):
	robot.goalPose.LH_roll = -data.data


def call_RH_pos_x(data):
	robot.goalPose.RH_pos_y = -data.data * presets.USER_MAPPING_MULTIPLIER

def call_RH_pos_y(data):
	robot.goalPose.RH_pos_z = (data.data + presets.OCULUS_ORIGIN_Y_ADJUSTMENT) * presets.USER_MAPPING_MULTIPLIER

def call_RH_pos_z(data):
	robot.goalPose.RH_pos_x = -data.data * presets.USER_MAPPING_MULTIPLIER

def call_RH_roll(data):
	robot.goalPose.RH_yaw = -data.data

def call_RH_pitch(data):
	robot.goalPose.RH_pitch = -data.data + RIGHT_ANGLE_RAD

def call_RH_yaw(data):
	robot.goalPose.RH_roll = -data.data