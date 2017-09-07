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
	lquat = tf.transformations.quaternion_from_euler(robotPose.LH_roll, robotPose.LH_pitch, robotPose.LH_yaw, 'sxyz')

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
					x=lquat[0],
					y=lquat[1],
					z=lquat[2],
					w=lquat[3],
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
		limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
		return limb_joints
	else:
		return baxter_interface.Limb(limb).joint_angles()
	return