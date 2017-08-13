#!/usr/bin/env python
# Import the necessary Python modules

# rospy - ROS Python API
import rospy

# baxter_interface - Baxter Python API
import baxter_interface

def main():

	# initialize our ROS node, registering it with the Master
	rospy.init_node("Hello_Baxter")

	# create an instance of baxter_interface's Limb class
	limb = baxter_interface.Limb("right")

	# get the right limb's current joint angles
	angles = limb.joint_angles()

	# print the current joint angles
	print angles

	# reassign new joint angles (all zeros) which we will later command to the limb
	angles['right_s0']=0.0
	angles['right_s1']=0.0
	angles['right_e0']=0.0
	angles['right_e1']=0.0
	angles['right_w0']=0.0
	angles['right_w1']=0.0
	angles['right_w2']=0.0

	# print the joint angle command
	print angles

	# move the right arm to those joint angles
	limb.move_to_joint_positions(angles)

	# Baxter wants to say hello, let's wave the arm

	# store the first wave position
	wave_1 = {'right_s0': -0.459, 'right_s1': -0.202, 'right_e0': 1.807, 'right_e1': 1.714, 'right_w0': -0.906, 'right_w1': -1.545, 'right_w2': -0.276}

	# store the second wave position
	wave_2 = {'right_s0': -0.395, 'right_s1': -0.202, 'right_e0': 1.831, 'right_e1': 1.981, 'right_w0': -1.979, 'right_w1': -1.100, 'right_w2': -0.448}

	# wave three times
	for _move in range(3):
		limb.move_to_joint_positions(wave_1)
		limb.move_to_joint_positions(wave_2)

	# quit
	quit()

if __name__ == '__main__':
	main()