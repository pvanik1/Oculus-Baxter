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

def add_roll(current_joint_angles, initial_angle, limb):
	if limb == 'right':
		wrist = 'right_w2'
		yaw = robot.goalPose.RH_yaw
	else:
		wrist = 'left_w2'
		yaw = robot.goalPose.LH_yaw
	angles=current_joint_angles
	if yaw + initial_angle[wrist] <= presets.WRIST_ROLL_MIN:
		angles[wrist] = presets.WRIST_ROLL_MIN		
	elif yaw + initial_angle[wrist] >= presets.WRIST_ROLL_MAX:
		angles[wrist] = presets.WRIST_ROLL_MAX
	else: 
		angles[wrist] = yaw + initial_angle[wrist]
	return angles

def roll(rolled_limb):
	global LH_initial_angle
	global RH_initial_angle
	if rolled_limb == 'left':
		limb = robot.left_limb
		mode = robot.LH_mode_roll
		rolling = LH_rolling
		wrist = 'left_w2'
	else:
		limb = robot.right_limb
		mode = robot.RH_mode_roll
		rolling = RH_rolling
		wrist = 'right_w2'

	currAngles = limb.joint_angles()
	currAnglesCopy = copy.deepcopy(currAngles)

	if rolled_limb == 'left':
		if rolling == False:
			LH_initial_angle = limb.joint_angles()
		currAnglesCopy = add_roll(currAnglesCopy, LH_initial_angle, 'left')
	else:
		if rolling == False:
			RH_initial_angle = limb.joint_angles()
		currAnglesCopy = add_roll(currAnglesCopy, RH_initial_angle, 'right')

	while ((abs(currAngles[wrist] - currAnglesCopy[wrist]) >= presets.PRECISION_TOLERANCE/2) & mode == True):
		limb.set_joint_positions(currAnglesCopy)
		currAngles = limb.joint_angles()		
	return

# Function to prevent repeating code for checking if left hand should be rolled
def check_roll_left():
	global LH_rolling
	if robot.LH_mode_roll == False:	
		LH_rolling = False
		robot.left_limb.set_joint_positions(iksolver.ik_solve('left', robot.goalPose))
	else:
		roll('left')
		LH_rolling = True
	return


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
	RH_rolling = False
	LH_rolling = False

	while not rospy.is_shutdown():

		while not robot.poseGoalReached():
			if robot.RH_mode_roll == False:
				RH_rolling = False
				robot.right_limb.set_joint_positions(iksolver.ik_solve('right', robot.goalPose))
				check_roll_left()
			else:
				roll('right')
				RH_rolling = True
				check_roll_left()
	# quit
	quit()