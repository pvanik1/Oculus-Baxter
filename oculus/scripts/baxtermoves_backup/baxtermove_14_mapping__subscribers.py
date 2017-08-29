#!/usr/bin/env python
"""
Subscriber ROS node which processes data from pose topics and uses
them to move the robot.
"""

# rospy - ROS Python API
import rospy

# baxter_interface - Baxter Python API
import baxter_interface

import argparse
import sys
from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Point,
	Quaternion,
)
from std_msgs.msg import (
	Header,
	Float64,
)
 
from baxter_core_msgs.srv import (
	SolvePositionIK,
	SolvePositionIKRequest,
)

from robotpose import RobotPose

import math

# Movement ranges (in meters) are determined by maximum distance of the end 
# effector from coordinate frame origin. East-west axis limb stretch.
USER_MOVEMENT_RANGE = 0.8
ROBOT_MOVEMENT_RANGE = 1.45

PRECISION_TOLERANCE = 0.1 # Threshold of accuracy between target pose and actual pose
PRECISION_MODE = False # 1:1 human:robot tracking of end effectors for precision tasks.
# Multiplier for mapping human figure onto the robot:
if (PRECISION_MODE == False):
	USER_MAPPING_MULTIPLIER = ROBOT_MOVEMENT_RANGE / USER_MOVEMENT_RANGE
else: USER_MAPPING_MULTIPLIER = 1

# Measurements to determine correct placement of the origin of Oculus Rift coordinate 
# frame for optimal immersion. Based on east-west axis limb stretch (i.e. parallel to floor).
EXTERNAL_CAM_TO_EYES_DISTANCE = 0.045 # Set to 0 if not mapping external camera onto robot
ROBOT_ORIGIN_TO_EYES_DISTANCE = 0.75 + EXTERNAL_CAM_TO_EYES_DISTANCE
ROBOT_STRETCHED_Z = 0.32 # Up-down axis coordinate of the end effector with limbs stretched
USER_STRETCHED_Z = abs(0.3) # Up-down axis coordinate of Oculus Touch with limbs stretched. Origin = Oculus Rift HMD.
def CALC_OFFSET():
	alpha=math.atan(ROBOT_STRETCHED_Z / ROBOT_MOVEMENT_RANGE)
	user_shoulder_to_origin_z = math.tan(alpha) * USER_MOVEMENT_RANGE
	return (user_shoulder_to_origin_z + USER_STRETCHED_Z)
OCULUS_ORIGIN_Y_ADJUSTMENT = CALC_OFFSET()

def ik_solve(limb, robotPose):
	ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
	iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
	ikreq = SolvePositionIKRequest()
	hdr = Header(stamp=rospy.Time.now(), frame_id='base')

	poses = {
		'left': PoseStamped(
			header=hdr,
			pose=Pose(
				position=Point(
					x=robotPose.LH_pos_x,
					y=robotPose.LH_pos_y,
					z=robotPose.LH_pos_z,
				),
				orientation=Quaternion(
					x=robotPose.LH_ori_x,
					y=robotPose.LH_ori_y,
					z=robotPose.LH_ori_z,
					w=robotPose.LH_ori_w,
				),
			),
		),
		'right':PoseStamped(
			header=hdr,
			pose=Pose(
				position=Point(
					x=robotPose.RH_pos_x,
					y=robotPose.RH_pos_y,
					z=robotPose.RH_pos_z,
				),
				orientation=Quaternion(
					x=robotPose.RH_ori_x,
					y=robotPose.RH_ori_y,
					z=robotPose.RH_ori_z,
					w=robotPose.RH_ori_w,
				),
			),
		),
	}
	ikreq.pose_stamp.append(poses[limb])

	try:
		rospy.wait_for_service(ns, 5.0)
		resp = iksvc(ikreq)
	except (rospy.ServiceException, rospy.ROSException), e:
		rospy.logerr("Service call failed: %s" % (e,))
		return

	if (resp.isValid[0]):
		#print("SUCCEESS - Valid Joint Solution Found:")
		# Format solution into Limb API-compatible dictionary
		limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
		return limb_joints
	else:
		#print("INVALID POSE - No Valid Joint Solution Found.")
		return ik_solve(limb, currentPose)
	return

# For testing purposes
def update():
	global goalPose
	global right_limb
	global left_limb

	goalPose.RH_pos_y += 0.12
	goalPose.LH_pos_y -= 0.07

	if (goalPose.RH_pos_y >= -0.1):
		print("Done :)")
		move_to_human_neutral()
		quit()

def create_subs():

	rospy.Subscriber('LH_pos_x', Float64, call_LH_pos_x)
	rospy.Subscriber('LH_pos_y', Float64, call_LH_pos_y)
	rospy.Subscriber('LH_pos_z', Float64, call_LH_pos_z)
	rospy.Subscriber('LH_ori_w', Float64, call_LH_ori_w)
	rospy.Subscriber('LH_ori_x', Float64, call_LH_ori_x)
	rospy.Subscriber('LH_ori_y', Float64, call_LH_ori_y)
	rospy.Subscriber('LH_ori_z', Float64, call_LH_ori_z)

	rospy.Subscriber('RH_pos_x', Float64, call_RH_pos_x)
	rospy.Subscriber('RH_pos_y', Float64, call_RH_pos_y)
	rospy.Subscriber('RH_pos_z', Float64, call_RH_pos_z)
	rospy.Subscriber('RH_ori_w', Float64, call_RH_ori_w)
	rospy.Subscriber('RH_ori_x', Float64, call_RH_ori_x)
	rospy.Subscriber('RH_ori_y', Float64, call_RH_ori_y)
	rospy.Subscriber('RH_ori_z', Float64, call_RH_ori_z)

# Callback functions that map Oculus pose to robot pose.
# Oculus coordinate system is different from Baxter's:
# Xbaxter = negative Zoculus
# Ybaxter = Xoculus
# Zbaxter = Yoculus
def call_LH_pos_x(data):
	global goalPose
	goalPose.LH_pos_y = data.data * USER_MAPPING_MULTIPLIER
	#rospy.loginfo("LH pos y : %f", goalPose.LH_pos_y)

def call_LH_pos_y(data):
	global goalPose
	goalPose.LH_pos_z = (data.data + OCULUS_ORIGIN_Y_ADJUSTMENT) * USER_MAPPING_MULTIPLIER
	#rospy.loginfo("LH pos z : %f", goalPose.LH_pos_z)

def call_LH_pos_z(data):
	global goalPose
	goalPose.LH_pos_x = -data.data  * USER_MAPPING_MULTIPLIER
	#rospy.loginfo("LH pos x : %f", goalPose.LH_pos_x)

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
	goalPose.RH_pos_y = data.data * USER_MAPPING_MULTIPLIER
	rospy.loginfo("RH pos y : %f", goalPose.RH_pos_y)
	#rospy.loginfo("%f", data.data)

def call_RH_pos_y(data):
	global goalPose
	goalPose.RH_pos_z = (data.data + OCULUS_ORIGIN_Y_ADJUSTMENT) * USER_MAPPING_MULTIPLIER
	rospy.loginfo("RH pos z : %f", goalPose.RH_pos_z)

def call_RH_pos_z(data):
	global goalPose
	goalPose.RH_pos_x = -data.data * USER_MAPPING_MULTIPLIER
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

def rockstar():
	global right_limb
	global left_limb

	right_angles = {'right_s0':0.5, 'right_s1':-0.7, 'right_e0':0.0, 'right_e1':0.0,
						'right_w0':0.0, 'right_w1':-0.3, 'right_w2':0.0}
	left_angles = {'left_s0':-0.5, 'left_s1':-0.7, 'left_e0':0.0, 'left_e1': 0.0,
						'left_w0': 0.0, 'left_w1':-0.3, 'left_w2':0.0}

	counter = 0
	while(counter < 100):
		right_limb.set_joint_positions(right_angles)
		rospy.sleep(0.01)
		left_limb.set_joint_positions(left_angles)
		rospy.sleep(0.01)
		counter+=1

def move_to_human_neutral():
	global right_limb
	global left_limb

	right_angles = {'right_s0':0.5, 'right_s1':-1, 'right_e0':0.0, 'right_e1':2.3,
						'right_w0':0.0, 'right_w1':-1, 'right_w2':0.0}
	left_angles = {'left_s0':-0.5, 'left_s1':-1, 'left_e0':0.0, 'left_e1': 2.3,
						'left_w0': 0.0, 'left_w1':-1, 'left_w2':0.0}

	counter = 0
	while(counter < 80):
		#cmp(right_angles, right_limb.joint_angles()) + cmp(left_angles, left_limb.joint_angles()) != 0):
		right_limb.set_joint_positions(right_angles)
		rospy.sleep(0.01)
		left_limb.set_joint_positions(left_angles)
		rospy.sleep(0.01)
		counter+=1

def goalReached():
	LH_x_GoalReached = (abs(left_limb.endpoint_pose()["position"].x - goalPose.LH_pos_x) <= PRECISION_TOLERANCE)
	LH_y_GoalReached = (abs(left_limb.endpoint_pose()["position"].y - goalPose.LH_pos_y) <= PRECISION_TOLERANCE)
	LH_z_GoalReached = (abs(left_limb.endpoint_pose()["position"].z - goalPose.LH_pos_z) <= PRECISION_TOLERANCE)

	RH_x_GoalReached = (abs(right_limb.endpoint_pose()["position"].x - goalPose.RH_pos_x) <= PRECISION_TOLERANCE)
	RH_y_GoalReached = (abs(right_limb.endpoint_pose()["position"].y - goalPose.RH_pos_y) <= PRECISION_TOLERANCE)
	RH_z_GoalReached = (abs(right_limb.endpoint_pose()["position"].z - goalPose.RH_pos_z) <= PRECISION_TOLERANCE)

	#if (LH_x_GoalReached & LH_y_GoalReached & LH_z_GoalReached & RH_x_GoalReached & RH_y_GoalReached & RH_z_GoalReached):
	if (RH_x_GoalReached & RH_y_GoalReached & RH_z_GoalReached):
		print("Goal Reached!")
		return True
	else:
		#print abs(left_limb.endpoint_pose()["position"].x - goalPose.LH_pos_x)
		#print abs(left_limb.endpoint_pose()["position"].y - goalPose.LH_pos_y)
		#print abs(left_limb.endpoint_pose()["position"].z - goalPose.LH_pos_z)
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


if __name__ == '__main__':
	# initialize the ROS node, registering it with the Master
	rospy.init_node("Baxter_Move_Subscribers")
	rs = baxter_interface.RobotEnable()
	rs.enable()

	# create an instance of baxter_interface's Limb class
	right_limb = baxter_interface.Limb("right")
	left_limb = baxter_interface.Limb("left")

	# initialize robot pose
	goalPose = RobotPose()
	currentPose = RobotPose()
	updateCurrentPose()

	left_limb.set_joint_position_speed(0.6)
	right_limb.set_joint_position_speed(0.6)

	# move to starting pose
	move_to_human_neutral()
	#rockstar()

	rospy.sleep(5)

	# initialise subscribers
	create_subs()

	#print(OCULUS_ORIGIN_Y_ADJUSTMENT)
	#print(currentPose.RH_pos_z)

	while not rospy.is_shutdown():
		while not goalReached():
			#right_limb.set_joint_positions(ik_solve('right', goalPose))
			right_limb.set_joint_positions(ik_solve('right', goalPose))
			rospy.sleep(0.01)
			#left_limb.set_joint_positions(ik_solve('left', goalPose))
			#left_limb.set_joint_positions(ik_solve('left', goalPose))
			#rospy.sleep(0.01)
			updateCurrentPose()
	# 	right_limb.move_to_joint_positions(ik_solve('right', goalPose), 0.2)
	# 	left_limb.move_to_joint_positions(ik_solve('left', goalPose), 0.2)
		print("SLEEPING ---------------------------")
		rospy.sleep(2)

	# quit
	quit()