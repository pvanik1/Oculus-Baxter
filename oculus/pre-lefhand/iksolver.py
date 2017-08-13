#!/usr/bin/env python
"""
Inverse kinematic solvers
"""
import rospy
import tf
from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Point,
	Quaternion,
)
from baxter_core_msgs.srv import (
	SolvePositionIK,
	SolvePositionIKRequest,
)
from std_msgs.msg import Header
import baxter_interface

def ik_solve(limb, robotPose):
	ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
	iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
	ikreq = SolvePositionIKRequest()
	hdr = Header(stamp=rospy.Time.now(), frame_id='base')

	rquat = tf.transformations.quaternion_from_euler(robotPose.RH_roll, robotPose.RH_pitch, robotPose.RH_yaw, 'sxyz')
	#TODO left hand

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
					x=rquat[0],
					y=rquat[1],
					z=rquat[2],
					w=rquat[3],
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
		#return ik_solve(limb, currentPose)
		return baxter_interface.Limb(limb).joint_angles()
	return




def ik_solve_roll(limb, robotPose):
	ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
	iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
	ikreq = SolvePositionIKRequest()
	hdr = Header(stamp=rospy.Time.now(), frame_id='right_gripper')

	rquat = tf.transformations.quaternion_from_euler(0, 0, robotPose.RH_yaw, 'sxyz')
	#TODO left hand

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
					x=0,
					y=0,
					z=0,
				),
				orientation=Quaternion(
					x=rquat[0],
					y=rquat[1],
					z=rquat[2],
					w=rquat[3],
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
		#return ik_solve(limb, currentPose)
		return baxter_interface.Limb(limb).joint_angles()
	return