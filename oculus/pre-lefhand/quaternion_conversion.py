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

import tf
import robotpose

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

		# Right hand stretched arms:
		# {'position': Point(x=0.06867847255642005, y=-1.2961428003191218, z=0.31596174778599195), 
		# 'orientation': Quaternion(x=0.4951974029638538, y=0.5012344755162788, z=-0.5029114409542016, w=0.500623226797503)}

		'right':PoseStamped(
			header=hdr,
			pose=Pose(
				position=Point(
					x=0.06867847255642005,
					y=-1.2961428003191218,
					z=0.31596174778599195,
				),
				orientation=Quaternion(
					x=0.4951974029638538,
					y=0.5012344755162788,
					z=0.5029114409542016,
					w=0.500623226797503,
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
	head = baxter_interface.Head()
	#head.set_pan(0)

	#print("Here are the current humanoidish pose angles", right_angles)

	right_angles['right_s0']=-0.7854
	right_angles['right_s1']=0*0.7854
	right_angles['right_e0']=4*0.7854
	right_angles['right_e1']=0*0.7854
	right_angles['right_w0']=0*0.7854
	right_angles['right_w1']=2*0.7854
	right_angles['right_w2']=0*0.7854

	left_angles['left_s0']=0.7854
	left_angles['left_s1']=0.0
	left_angles['left_e0']=0.0
	left_angles['left_e1']=0.0
	left_angles['left_w0']=0.0
	left_angles['left_w1']=0.0
	left_angles['left_w2']=0.0

	# angles = {'right_s0': -0.7854, 'right_s1': 0, 'right_e0': 0, 'right_e1': 2*0.7854, 
	# 			'right_w0': -2*0.7854, 'right_w1': -2*0.7854, 'right_w2': 0}

	# move the right arm to those joint angles
	right_limb.set_joint_position_speed(0.4)

	#right_limb.move_to_joint_positions(right_angles)

	#right_limb.move_to_joint_positions(ik_solve('right'))

	quaternion = right_limb.endpoint_pose()["orientation"]
	euler = tf.transformations.euler_from_quaternion(quaternion)
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]

	print("Quaternion to euler:", roll*57.2958, pitch*57.2958, yaw*57.2958)
	
	#print(left_limb.endpoint_pose())
	# print("Right limb pose:")
	# print(right_limb.endpoint_pose())

	# quit
	quit()

if __name__ == '__main__':
	main()