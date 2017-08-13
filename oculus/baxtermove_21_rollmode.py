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
	if robot.goalPose.RH_yaw + initial_angle['right_w2'] <= presets.WRIST_ROLL_MIN:
		angles['right_w2'] = presets.WRIST_ROLL_MIN		
	elif robot.goalPose.RH_yaw + initial_angle['right_w2'] >= presets.WRIST_ROLL_MAX:
		angles['right_w2'] = presets.WRIST_ROLL_MAX
	else: 
		angles['right_w2'] = robot.goalPose.RH_yaw + initial_angle['right_w2']
	return angles

if __name__ == '__main__':
	# Initialize the ROS node and enable robot
	rospy.init_node("Oculus_Baxter")
	rospy.sleep(0.5)
	rs = baxter_interface.RobotEnable()
	rs.enable()

	# Initialise robot interface
	robot.initialise()

	# Move to starting pose
	robot.reset_pose()

	# Initialise subscribers
	subscribers.create_subs()

	# Flag for "roll mode"
	rolling = False

	while not rospy.is_shutdown():
		print("outside of posegoalcreached")
		print (robot.RH_mode_roll)

		while not robot.poseGoalReached():
			if robot.RH_mode_roll == False:
				rolling = False
				robot.right_limb.set_joint_positions(iksolver.ik_solve('right', robot.goalPose))	
				#left_limb.set_joint_positions(ik_solve('left', goalPose))
				#robot.updateCurrentPose()
				if (presets.FIXED_ORIENTATION):
					robot.fixOrientation()

			else:
				print("in roll mode")
				currAngles = robot.right_limb.joint_angles()
				currAnglesCopy = copy.deepcopy(currAngles)
				if rolling == False:
					initial_angle = robot.right_limb.joint_angles()
				currAnglesCopy = add_roll(currAnglesCopy, initial_angle)

				while ((abs(currAngles['right_w2'] - currAnglesCopy['right_w2']) >= presets.PRECISION_TOLERANCE) & robot.RH_mode_roll == True):
					robot.right_limb.set_joint_positions(currAnglesCopy)
					currAngles = robot.right_limb.joint_angles()
				
				rolling = True
	# quit
	quit()