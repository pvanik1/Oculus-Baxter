#!/usr/bin/env python
"""
Subscriber ROS node which processes data from pose topics and uses
them to move the robot.
"""
import rospy
import baxter_interface

import presets
import fun
import iksolver
import robot
import subscribers

import copy

def add_roll(current_joint_angles, initial_angle):
	angles=current_joint_angles
	print("angles at start", angles['right_w2'])
	print("target angle", robot.goalPose.RH_yaw)
	if robot.goalPose.RH_yaw + initial_angle['right_w2'] <= presets.WRIST_ROLL_MIN:
		angles['right_w2'] = presets.WRIST_ROLL_MIN		
		#print("MIN MIN")
	elif robot.goalPose.RH_yaw + initial_angle['right_w2'] >= presets.WRIST_ROLL_MAX:
		angles['right_w2'] = presets.WRIST_ROLL_MAX
		#print("MAX MAX")
	else: 
		angles['right_w2'] = robot.goalPose.RH_yaw + initial_angle['right_w2']
	return angles

if __name__ == '__main__':
	# Initialize the ROS node and enable robot
	rospy.init_node("Oculus_Baxter")
	rs = baxter_interface.RobotEnable()
	rs.enable()

	# Initialise robot interface
	robot.initialise()

	# Move to starting pose
	robot.reset_pose()

	# Initialise subscribers
	subscribers.create_subs()
	
	firstime = False

	while not rospy.is_shutdown():
		 

		if not (robot.RH_mode_roll):
			firstime = False
			while not robot.poseGoalReached():
				robot.right_limb.set_joint_positions(iksolver.ik_solve('right', robot.goalPose))	
				#left_limb.set_joint_positions(ik_solve('left', goalPose))
				robot.updateCurrentPose()

				if (presets.FIXED_ORIENTATION):
					robot.fixOrientation()
		else:
			currAngles = robot.right_limb.joint_angles()
			currAngles2 = copy.deepcopy(currAngles)
			if firstime == False:
				initial_angle = robot.right_limb.joint_angles()
			currAngles2 = add_roll(currAngles2, initial_angle)

			while ((abs(currAngles['right_w2'] - currAngles2['right_w2']) >= presets.PRECISION_TOLERANCE) & robot.RH_mode_roll == True):
				robot.right_limb.set_joint_positions(currAngles2)
				currAngles = robot.right_limb.joint_angles()
			
			print("hotovo")
			firstime = True
	# quit
	quit()