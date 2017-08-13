#!/usr/bin/env python
"""
Presets defining user-to-robot mapping and user preferences.
"""
import rospy
import math
from sensor_msgs.msg import Image
import cv2
import cv_bridge

# Boolean whether left and right Rethink electric grippers are connected
LEFT_GRIPPER_CONNECTED = True
RIGHT_GRIPPER_CONNECTED = False

# Robot joint speed (0.0 - 1.0)
JOINT_SPEED = 1.0

# Velocity of the robot gripper (0.0 - 100.0)
GRIP_VELOCITY = 100

# Fixes the orientation of the end effector orientation in a certain direction. May allow better mobility.
# Available directions: forward, down
FIXED_ORIENTATION = False
FIXED_ORIENTATION_DIRECTION = 'forward'

# Movement ranges (in meters) are determined by maximum distance of the end 
# effector from coordinate origin in an east-west axis limb stretch.
USER_MOVEMENT_RANGE = 0.8
ROBOT_MOVEMENT_RANGE = 1.45

# Threshold of accuracy between target pose and actual pose
PRECISION_TOLERANCE = 0.05 

# Multiplier for mapping user's figure onto the robot:
USER_MAPPING_MULTIPLIER = ROBOT_MOVEMENT_RANGE / USER_MOVEMENT_RANGE

# Measurements to determine correct placement of the origin of Oculus Rift coordinate 
# frame. Based on east-west axis limb stretch
ROBOT_STRETCHED_Z = 0.32 # Up-down axis coordinate of the end effector with limbs stretched
USER_STRETCHED_Z = abs(-0.3) # Up-down axis coordinate of Oculus Touch with limbs stretched. Origin = Oculus Rift HMD.
def CALC_OFFSET():
	alpha=math.atan(ROBOT_STRETCHED_Z / ROBOT_MOVEMENT_RANGE)
	user_shoulder_to_origin_z = math.tan(alpha) * USER_MOVEMENT_RANGE
	return (user_shoulder_to_origin_z + USER_STRETCHED_Z)
OCULUS_ORIGIN_Y_ADJUSTMENT = CALC_OFFSET()

# System path to the image displayed on Baxter's display
FACE_IMAGE_PATH = "/home/petervanik/catkin_ws/src/oculus/scripts/ironbaxter.jpg"

# Minimum and maximum rotation values of the robot in radians
ROBOT_HEAD_PAN_MIN = -1.3
ROBOT_HEAD_PAN_MAX = 1.3
WRIST_ROLL_MIN = -3.059
WRIST_ROLL_MAX = 3.059

# End effector orientations when pointing in various directions
def getFixedOrientation():
	orientations = {
		'down': {
			"x":0,
			"y":1,
			"z":0,
			"w":0
		},
		'forward': {
			"x":0.6998733236487326,
			"y":0.04441122697756609,
			"z":0.7127345910262994,
			"w":-0.014641602296105763
		}
	}
	return orientations.FIXED_ORIENTATION_DIRECTION


# Send image located at specified path to Baxter
def display_image():
	img = cv2.imread(FACE_IMAGE_PATH)
	msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
	pub = rospy.Publisher('/robot/xdisplay', Image, queue_size=100, latch=True)
	pub.publish(msg)