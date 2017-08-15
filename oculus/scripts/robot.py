#!/usr/bin/env python
"""
Robot's pose and limb interface initialisation.
"""
import baxter_interface
import presets

class RobotPose:
	def __init__(self):
		self.RH_pos_x = 0.656982770038
		self.RH_pos_y = -0.852598021641
		self.RH_pos_z = 0.0388609422173
		self.RH_ori_w = 0.261868353356
		self.RH_ori_x = 0.367048116303
		self.RH_ori_y = 0.885911751787
		self.RH_ori_z = -0.108908281936

		self.RH_roll = 0.0
		self.RH_pitch = 1.5708
		self.RH_yaw = 0.0

		self.LH_pos_x = 0.657579481614
		self.LH_pos_y = 0.851981417433
		self.LH_pos_z = 0.0388352386502
		self.LH_ori_w = 0.262162481772
		self.LH_ori_x = -0.366894936773
		self.LH_ori_y = 0.885980397775
		self.LH_ori_z = 0.108155782462

		self.LH_roll = 0.0
		self.LH_pitch = 1.5708
		self.LH_yaw = 0.0

def initialise(pose):
	global right_limb
	global left_limb
	global right_gripper
	global left_gripper
	global head
	global goalPose
	global currentPose
	global RH_mode_roll
	global LH_mode_roll
	
	# Initialise the robot's interface and pose
	right_limb = baxter_interface.Limb("right")
	left_limb = baxter_interface.Limb("left")
	right_gripper = baxter_interface.Gripper('right')
	left_gripper = baxter_interface.Gripper('left')
	head = baxter_interface.Head()

	# In roll mode, the user is not controlling the position or orientation
	# of the end effector, just the roll of the wrist.
	RH_mode_roll = False
	LH_mode_roll = False

	# Set limb and gripper parameters
	right_limb.set_joint_position_speed(presets.JOINT_SPEED)
	left_limb.set_joint_position_speed(presets.JOINT_SPEED)

	if presets.RIGHT_GRIPPER_CONNECTED:
		right_gripper.set_velocity(presets.GRIP_VELOCITY)
		right_gripper.calibrate()

	if presets.LEFT_GRIPPER_CONNECTED:
		left_gripper.set_velocity(presets.GRIP_VELOCITY)
		left_gripper.calibrate()

	# Set up head
	head.set_pan(0)
	head.command_nod()
	presets.display_image()

	# Initialise current and goal pose of the robot
	goalPose = RobotPose()
	currentPose = RobotPose()
	reset_pose(pose)
	updateCurrentPose()	
	

def updateCurrentPose():
	currentPose.RH_pos_x = right_limb.endpoint_pose()["position"].x
	currentPose.RH_pos_y = right_limb.endpoint_pose()["position"].y
	currentPose.RH_pos_z = right_limb.endpoint_pose()["position"].z
	currentPose.RH_ori_w = right_limb.endpoint_pose()["orientation"].w
	currentPose.RH_ori_x = right_limb.endpoint_pose()["orientation"].x
	currentPose.RH_ori_y = right_limb.endpoint_pose()["orientation"].y
	currentPose.RH_ori_z = right_limb.endpoint_pose()["orientation"].z

	currentPose.LH_pos_x = left_limb.endpoint_pose()["position"].x
	currentPose.LH_pos_y = left_limb.endpoint_pose()["position"].y
	currentPose.LH_pos_z = left_limb.endpoint_pose()["position"].z
	currentPose.LH_ori_w = left_limb.endpoint_pose()["orientation"].w
	currentPose.LH_ori_x = left_limb.endpoint_pose()["orientation"].x
	currentPose.LH_ori_y = left_limb.endpoint_pose()["orientation"].y
	currentPose.LH_ori_z = left_limb.endpoint_pose()["orientation"].z

# Reset Baxter to a specified neutral pose:
#	"baxter" - for pick and place tasks
#	"human" - for mimicking human motion
def reset_pose(pose):
	if pose == "human":
		rangles = {'right_s0': -0.30181072001645515, 'right_s1': -0.17180584824316633, 'right_w0': -1.3230584295511694, 
					'right_w1': -1.5715633171886063, 'right_w2': -0.5250049246537829, 'right_e0': 0.24313595487983808, 'right_e1': 2.16483038690329}
		langles = {'left_w0': 1.2655341500054664, 'left_w1': -1.4442429117941171, 'left_w2': 0.4555922940019679, 
				'left_e0': -0.22971362298584072, 'left_e1': 2.2595537005552147, 'left_s0': 0.33555829734993425, 'left_s1': -0.33670878294084833}
	else:
		rangles = {'right_s0': 0.01572330307582549, 'right_s1': -1.3222914391572267, 'right_w0': -0.055223308363874894, 'right_w1': 0.4640291883353377, 
					'right_w2': -0.0030679615757708274, 'right_e0': 0.3976845192592935, 'right_e1': 2.1533255309941497}

		langles = {'left_w0': 0.07784952498518474, 'left_w1': 0.44447093328979864, 'left_w2': 0.0011504855909140602, 'left_e0': -0.5898156129419416, 
					'left_e1': 2.137602227918324, 'left_s0': 0.048703890015361885, 'left_s1': -1.3794322235059584}


	right_limb.set_joint_position_speed(1)
	right_limb.move_to_joint_positions(rangles)
	right_limb.set_joint_position_speed(presets.JOINT_SPEED)
	
	left_limb.set_joint_position_speed(1)
	left_limb.move_to_joint_positions(langles)
	left_limb.set_joint_position_speed(presets.JOINT_SPEED)

	updateCurrentPose()

# Resets the orientation of the goal pose to the specified fixed orientation
def fixOrientation():
	orientation = presets.getFixedOrientation()
	goalPose.RH_ori_x =  orientation["x"]
	goalPose.RH_ori_y =  orientation["y"]
	goalPose.RH_ori_z =  orientation["z"]
	goalPose.RH_ori_w =  orientation["w"]

	goalPose.LH_ori_x =  orientation["x"]
	goalPose.LH_ori_y =  orientation["y"]
	goalPose.LH_ori_z =  orientation["z"]
	goalPose.LH_ori_w =  orientation["w"]


def poseGoalReached():
	updateCurrentPose()

	RH_pos_x_GoalReached = (abs(currentPose.RH_pos_x - goalPose.RH_pos_x) <= presets.PRECISION_TOLERANCE)
	RH_pos_y_GoalReached = (abs(currentPose.RH_pos_y - goalPose.RH_pos_y) <= presets.PRECISION_TOLERANCE)
	RH_pos_z_GoalReached = (abs(currentPose.RH_pos_z - goalPose.RH_pos_z) <= presets.PRECISION_TOLERANCE)

	# RH_pos_x_GoalReached = (abs(right_limb.endpoint_pose()["position"].x - goalPose.RH_pos_x) <= presets.PRECISION_TOLERANCE)
	# RH_pos_y_GoalReached = (abs(right_limb.endpoint_pose()["position"].y - goalPose.RH_pos_y) <= presets.PRECISION_TOLERANCE)
	# RH_pos_z_GoalReached = (abs(right_limb.endpoint_pose()["position"].z - goalPose.RH_pos_z) <= presets.PRECISION_TOLERANCE)

	RH_ori_w_GoalReached = (abs(currentPose.RH_ori_w - goalPose.RH_ori_w) <= presets.PRECISION_TOLERANCE)
	RH_ori_x_GoalReached = (abs(currentPose.RH_ori_x - goalPose.RH_ori_x) <= presets.PRECISION_TOLERANCE)
	RH_ori_y_GoalReached = (abs(currentPose.RH_ori_y - goalPose.RH_ori_y) <= presets.PRECISION_TOLERANCE)
	RH_ori_z_GoalReached = (abs(currentPose.RH_ori_z - goalPose.RH_ori_z) <= presets.PRECISION_TOLERANCE)

	LH_pos_x_GoalReached = (abs(currentPose.LH_pos_x - goalPose.LH_pos_x) <= presets.PRECISION_TOLERANCE)
	LH_pos_y_GoalReached = (abs(currentPose.LH_pos_y - goalPose.LH_pos_y) <= presets.PRECISION_TOLERANCE)
	LH_pos_z_GoalReached = (abs(currentPose.LH_pos_z - goalPose.LH_pos_z) <= presets.PRECISION_TOLERANCE)

	LH_ori_w_GoalReached = (abs(currentPose.LH_ori_w - goalPose.LH_ori_w) <= presets.PRECISION_TOLERANCE)
	LH_ori_x_GoalReached = (abs(currentPose.LH_ori_x - goalPose.LH_ori_x) <= presets.PRECISION_TOLERANCE)
	LH_ori_y_GoalReached = (abs(currentPose.LH_ori_y - goalPose.LH_ori_y) <= presets.PRECISION_TOLERANCE)
	LH_ori_z_GoalReached = (abs(currentPose.LH_ori_z - goalPose.LH_ori_z) <= presets.PRECISION_TOLERANCE)

	if (RH_pos_x_GoalReached & RH_pos_y_GoalReached & RH_pos_z_GoalReached &
			RH_ori_w_GoalReached & RH_ori_x_GoalReached & RH_ori_y_GoalReached & RH_ori_z_GoalReached):
	# if (RH_pos_x_GoalReached & RH_pos_y_GoalReached & RH_pos_z_GoalReached &
	# 	RH_ori_w_GoalReached & RH_ori_x_GoalReached & RH_ori_y_GoalReached & RH_ori_z_GoalReached):
		# LH_pos_x_GoalReached & LH_pos_y_GoalReached & LH_pos_z_GoalReached &
		# LH_ori_w_GoalReached & LH_ori_x_GoalReached & LH_ori_y_GoalReached & LH_ori_z_GoalReached):
		return True
	else:
		return False