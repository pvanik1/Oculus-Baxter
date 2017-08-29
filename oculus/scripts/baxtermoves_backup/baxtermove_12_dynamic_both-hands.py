#!/usr/bin/env python
# Import the necessary Python modules

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
from std_msgs.msg import Header
 
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

def update():
	global goalPose
	global right_limb

	goalPose.RH_pos_y += 0.12
	goalPose.LH_pos_y -= 0.07

	if (goalPose.RH_pos_y >= -0.1):
		print("Done :)")
		set_to_human_neutral()
		quit()

def peace():
	global right_limb
	global left_limb

	right_angles = right_limb.joint_angles()
	left_angles = left_limb.joint_angles()

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

	right_limb.move_to_joint_positions(right_angles)
	left_limb.move_to_joint_positions(left_angles)

def set_to_human_neutral():
	global right_limb
	global left_limb

	right_angles = right_limb.joint_angles()
	left_angles = left_limb.joint_angles()

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

	right_limb.move_to_joint_positions(right_angles)
	left_limb.move_to_joint_positions(left_angles)

if __name__ == '__main__':
	# initialize the ROS node, registering it with the Master
	rospy.init_node("Baxter_Move_Dynamic")

	# initialise robot pose
	goalPose = RobotPose()

	# create an instance of baxter_interface's Limb class
	right_limb = baxter_interface.Limb("right")
	left_limb = baxter_interface.Limb("left")

	while not rospy.is_shutdown():
		update()
		print(goalPose.RH_pos_y)
		right_limb.set_joint_positions(ik_solve('right', goalPose))
		left_limb.set_joint_positions(ik_solve('left', goalPose))
		rospy.sleep(0.1)

	# quit
	quit()