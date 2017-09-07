#!/usr/bin/env python
"""
Robot's pose and limb interface initialisation.
"""
import baxter_interface
import presets

class RobotPose:
	def __init__(self):
		# Initial values for "human" pose
		self.RH_pos_x = 0.6342171236731038
		self.RH_pos_y = -0.36043150138041063
		self.RH_pos_z = 0.09859203828689367
		self.RH_ori_w = 0.7455594233663007
		self.RH_ori_x = -0.12293476834953236
		self.RH_ori_y = 0.6495988568385836
		self.RH_ori_z = 0.08396138490140141

		self.RH_roll = -30.939090668751863
		self.RH_pitch = 81.58539975618045
		self.RH_yaw = -14.12142637160184

		self.LH_pos_x = 0.6329613779972973
		self.LH_pos_y = 0.38766854601727874
		self.LH_pos_z = 0.09283436065295993
		self.LH_ori_w = 0.6965639109812897
		self.LH_ori_x = 0.1279323909724876
		self.LH_ori_y = 0.7019381984496922
		self.LH_ori_z = -0.07559620900358009

		self.LH_roll = -30.939090668751863
		self.LH_pitch = 81.58539975618045
		self.LH_yaw = -14.12142637160184

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

	# Alternative pose with hands pointing downward:
	# elif pose == "baxter":
	# 	rangles = {'right_s0': 0.01572330307582549, 'right_s1': -1.3222914391572267, 'right_w0': -0.055223308363874894, 'right_w1': 0.4640291883353377, 
	# 				'right_w2': -0.0030679615757708274, 'right_e0': 0.3976845192592935, 'right_e1': 2.1533255309941497}

	# 	langles = {'left_w0': 0.07784952498518474, 'left_w1': 0.44447093328979864, 'left_w2': 0.0011504855909140602, 'left_e0': -0.5898156129419416, 
	# 				'left_e1': 2.137602227918324, 'left_s0': 0.048703890015361885, 'left_s1': -1.3794322235059584}
	
	else: return

	right_limb.set_joint_position_speed(1)
	right_limb.move_to_joint_positions(rangles)
	right_limb.set_joint_position_speed(presets.JOINT_SPEED)
	
	left_limb.set_joint_position_speed(1)
	left_limb.move_to_joint_positions(langles)
	left_limb.set_joint_position_speed(presets.JOINT_SPEED)

	updateCurrentPose()

def poseGoalReached():
	updateCurrentPose()

	RH_pos_x_GoalReached = (abs(currentPose.RH_pos_x - goalPose.RH_pos_x) <= presets.PRECISION_TOLERANCE)
	RH_pos_y_GoalReached = (abs(currentPose.RH_pos_y - goalPose.RH_pos_y) <= presets.PRECISION_TOLERANCE)
	RH_pos_z_GoalReached = (abs(currentPose.RH_pos_z - goalPose.RH_pos_z) <= presets.PRECISION_TOLERANCE)

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
			RH_ori_w_GoalReached & RH_ori_x_GoalReached & RH_ori_y_GoalReached & RH_ori_z_GoalReached &
		LH_pos_x_GoalReached & LH_pos_y_GoalReached & LH_pos_z_GoalReached &
			LH_ori_w_GoalReached & LH_ori_x_GoalReached & LH_ori_y_GoalReached & LH_ori_z_GoalReached):
		return True
	else:
		return False