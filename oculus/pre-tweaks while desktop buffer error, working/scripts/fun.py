#!/usr/bin/env python
"""
Custom preset poses. Map them to any of the controller buttons.
"""
import robot

def rockstar():

	right_angles = {'right_s0':0.5, 'right_s1':-0.7, 'right_e0':0.0, 'right_e1':0.0,
						'right_w0':0.0, 'right_w1':-0.3, 'right_w2':0.0}
	left_angles = {'left_s0':-0.5, 'left_s1':-0.7, 'left_e0':0.0, 'left_e1': 0.0,
						'left_w0': 0.0, 'left_w1':-0.3, 'left_w2':0.0}

	robot.right_limb.move_to_joint_positions(right_angles)
	robot.left_limb.move_to_joint_positions(left_angles)
	robot.head.command_nod()