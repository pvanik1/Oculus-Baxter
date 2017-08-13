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

def ik_solve(limb):
	ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
	iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
	ikreq = SolvePositionIKRequest()
	hdr = Header(stamp=rospy.Time.now(), frame_id='base')

	poses = {
		'left': PoseStamped(
			header=hdr,
			pose=Pose(
				position=Point(
					x=0.657579481614,
					y=0.851981417433,
					z=0.0388352386502,
				),
				orientation=Quaternion(
					x=-0.366894936773,
					y=0.885980397775,
					z=0.108155782462,
					 w=0.262162481772,
				),
			),
		),
		'right':PoseStamped(
			header=hdr,
			pose=Pose(
				position=Point(
					x=0.656982770038,
					y=-0.852598021641,
					z=0.0388609422173,
				),
				orientation=Quaternion(
					x=0.367048116303,
					y=0.885911751787,
					z=-0.108908281936,
					w=0.261868353356,
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


def main():

	# initialize our ROS node, registering it with the Master
	rospy.init_node("Baxter_Move")

	# create an instance of baxter_interface's Limb class
	right_limb = baxter_interface.Limb("right")
	left_limb = baxter_interface.Limb("left")

	# get the right limb's current joint angles
	right_angles = right_limb.joint_angles()
	left_angles = left_limb.joint_angles()

	# reassign new joint angles (all zeros) which we will later command to the limb
	right_angles['right_s0']=0.0
	right_angles['right_s1']=0.0
	right_angles['right_e0']=0.0
	right_angles['right_e1']=0.0
	right_angles['right_w0']=0.0
	right_angles['right_w1']=0.0
	right_angles['right_w2']=0.0

	left_angles['left_s0']=0.0
	left_angles['left_s1']=0.0
	left_angles['left_e0']=0.0
	left_angles['left_e1']=0.6
	left_angles['left_w0']=0.0
	left_angles['left_w1']=0.0
	left_angles['left_w2']=0.0

	# move the right arm to those joint angles
	right_limb.move_to_joint_positions(right_angles)
	left_limb.move_to_joint_positions(left_angles) #TODO delete this afterwards

	right_limb.move_to_joint_positions(ik_solve('right'))
	left_limb.move_to_joint_positions(ik_solve('left'))

	# quit
	quit()

if __name__ == '__main__':
	main()