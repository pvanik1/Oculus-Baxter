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
		return 1

	if (resp.isValid[0]):
		print("SUCCEESS - Valid Joint Solution Found:")
		 # Format solution into Limb API-compatible dictionary
		limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
		return limb_joints
	else:
		print("INVALID POSE - No Valid Joint Solution Found.")

	return 0

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

	print("BREAK 99999")
	

# Callback functions

def call_LH_pos_x(data):
	global goalPose
	goalPose.LH_pos_x = data.data
	rospy.loginfo("LH pos x : %f", goalPose.LH_pos_x)
	#rospy.loginfo("%f", data.data)

def call_LH_pos_y(data):
	global goalPose
	goalPose.LH_pos_y = data.data
	rospy.loginfo("LH pos y : %f", goalPose.LH_pos_y)

def call_LH_pos_z(data):
	global goalPose
	goalPose.LH_pos_z = data.data
	rospy.loginfo("LH pos z : %f", goalPose.LH_pos_z)

def call_LH_ori_w(data):
	global goalPose
	goalPose.LH_ori_w = data.data
	rospy.loginfo("LH ori w : %f", goalPose.LH_ori_w)

def call_LH_ori_x(data):
	global goalPose
	goalPose.LH_ori_x = data.data
	rospy.loginfo("LH ori x : %f", goalPose.LH_ori_x)

def call_LH_ori_y(data):
	global goalPose
	goalPose.LH_ori_y = data.data
	rospy.loginfo("LH ori y : %f", goalPose.LH_ori_y)

def call_LH_ori_z(data):
	global goalPose
	goalPose.LH_ori_z = data.data
	rospy.loginfo("LH ori z : %f", goalPose.LH_ori_z)

def call_RH_pos_x(data):
	global goalPose
	goalPose.RH_pos_x = data.data
	rospy.loginfo("RH pos x : %f", goalPose.RH_pos_x)
	#rospy.loginfo("%f", data.data)

def call_RH_pos_y(data):
	global goalPose
	goalPose.RH_pos_y = data.data
	rospy.loginfo("RH pos y : %f", goalPose.RH_pos_y)

def call_RH_pos_z(data):
	global goalPose
	goalPose.RH_pos_z = data.data
	rospy.loginfo("RH pos z : %f", goalPose.RH_pos_z)

def call_RH_ori_w(data):
	global goalPose
	goalPose.RH_ori_w = data.data
	rospy.loginfo("RH ori w : %f", goalPose.RH_ori_w)

def call_RH_ori_x(data):
	global goalPose
	goalPose.RH_ori_x = data.data
	rospy.loginfo("RH ori x : %f", goalPose.RH_ori_x)

def call_RH_ori_y(data):
	global goalPose
	goalPose.RH_ori_y = data.data
	rospy.loginfo("RH ori y : %f", goalPose.RH_ori_y)

def call_RH_ori_z(data):
	global goalPose
	goalPose.RH_ori_z = data.data
	rospy.loginfo("RH ori z : %f", goalPose.RH_ori_z)

def rockstar():
	
	print("holaaa senor")
	right_angles={}
	left_angles={}

	right_angles['right_s0']=0.5
	right_angles['right_s1']=-0.7
	right_angles['right_e0']=0.0
	right_angles['right_e1']=0.0
	right_angles['right_w0']=0.0
	right_angles['right_w1']=-0.3
	right_angles['right_w2']=0.0

	left_angles['left_s0']=-0.5
	left_angles['left_s1']=-0.7
	left_angles['left_e0']=0.0
	left_angles['left_e1']=0.0
	left_angles['left_w0']=0.0
	left_angles['left_w1']=-0.3	
	left_angles['left_w2']=0.0

	right_limb.move_to_joint_positions(right_angles, 5, 0.1)
	left_limb.move_to_joint_positions(left_angles, 5, 0.1)

	right_limb.move_to_joint_positions(right_angles, 5, 0.1)
	left_limb.move_to_joint_positions(left_angles, 5, 0.1)
	right_limb.move_to_joint_positions(right_angles, 5, 0.1)
	left_limb.move_to_joint_positions(left_angles, 5, 0.1)
	right_limb.move_to_joint_positions(right_angles, 5, 0.1)
	left_limb.move_to_joint_positions(left_angles, 5, 0.1)
	right_limb.move_to_joint_positions(right_angles, 5, 0.1)
	left_limb.move_to_joint_positions(left_angles, 5, 0.1)
	right_limb.move_to_joint_positions(right_angles, 5, 0.1)
	left_limb.move_to_joint_positions(left_angles, 5, 0.1)
	right_limb.move_to_joint_positions(right_angles, 5, 0.1)
	left_limb.move_to_joint_positions(left_angles, 5, 0.1)
	

def move_to_human_neutral():
	#global right_limb
	#global left_limb
	right_angles = {}
	left_angles = {}

	right_angles['right_s0']=0.5
	right_angles['right_s1']=-1
	right_angles['right_e0']=0.0
	right_angles['right_e1']=2.3
	right_angles['right_w0']=0.0
	right_angles['right_w1']=-1
	right_angles['right_w2']=0.0

	left_angles['left_s0']=-0.5
	left_angles['left_s1']=-1
	left_angles['left_e0']=0.0
	left_angles['left_e1']=2.3
	left_angles['left_w0']=0.0
	left_angles['left_w1']=-1
	left_angles['left_w2']=0.0

	# in IK version, this is done by comparing taget pose and goal pose, e.g. compPose(goal, curr)
	counter = 0
	while(counter < 10):
		#cmp(right_angles, right_limb.joint_angles()) + cmp(left_angles, left_limb.joint_angles()) != 0):
		right_limb.set_joint_positions(right_angles)
		rospy.sleep(0.12)
		left_limb.set_joint_positions(left_angles)
		rospy.sleep(0.12)
		print(right_limb.joint_angles())
		print(left_limb.joint_angles())
		counter+=1



if __name__ == '__main__':
	# initialize the ROS node, registering it with the Master
	rospy.init_node("Baxter_Move_Subscribers")
	# rs = baxter_interface.RobotEnable()
	# rs.enable()

	# initialize robot pose
	goalPose = RobotPose()
	
	# create an instance of baxter_interface's Limb class
	right_limb = baxter_interface.Limb("right")
	left_limb = baxter_interface.Limb("left")

	right_limb.set_joint_position_speed(1)
	left_limb.set_joint_position_speed(1)

	rockstar()
	move_to_human_neutral()
	

	# initialise subscribers
	create_subs()

	# while not rospy.is_shutdown():
	# 	update()
	# 	right_limb.move_to_joint_positions(ik_solve('right', goalPose), 0.5)
	# 	left_limb.move_to_joint_positions(ik_solve('left', goalPose), 0.5)
	
	quit()