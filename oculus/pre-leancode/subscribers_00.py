#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

#Callback function template
#def callback(data):
# 	rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.data)

# Left hand

def call_LH_pos_x(data):
	global LH_pos_x
	LH_pos_x = data.data
	rospy.loginfo("LH pos x : %f", LH_pos_x)
	#rospy.loginfo("%f", data.data)

def call_LH_pos_y(data):
	global LH_pos_y
	LH_pos_y = data.data
	rospy.loginfo("LH pos y : %f", LH_pos_y)

def call_LH_pos_z(data):
	global LH_pos_z
	LH_pos_z = data.data
	rospy.loginfo("LH pos z : %f", LH_pos_z)

def call_LH_ori_w(data):
	global LH_ori_w
	LH_ori_w = data.data
	rospy.loginfo("LH ori w : %f", LH_ori_w)

def call_LH_ori_x(data):
	global LH_ori_x
	LH_ori_x = data.data
	rospy.loginfo("LH ori x : %f", LH_ori_x)

def call_LH_ori_y(data):
	global LH_ori_y
	LH_ori_y = data.data
	rospy.loginfo("LH ori y : %f", LH_ori_y)

def call_LH_ori_z(data):
	global LH_ori_z
	LH_ori_z = data.data
	rospy.loginfo("LH ori z : %f", LH_ori_z)

# Right hand

def call_RH_pos_x(data):
	global RH_pos_x
	RH_pos_x = data.data
	rospy.loginfo("RH pos x : %f", RH_pos_x)
	#rospy.loginfo("%f", data.data)

def call_RH_pos_y(data):
	global RH_pos_y
	RH_pos_y = data.data
	rospy.loginfo("RH pos y : %f", RH_pos_y)

def call_RH_pos_z(data):
	global RH_pos_z
	RH_pos_z = data.data
	rospy.loginfo("RH pos z : %f", RH_pos_z)

def call_RH_ori_w(data):
	global RH_ori_w
	RH_ori_w = data.data
	rospy.loginfo("RH ori w : %f", RH_ori_w)

def call_RH_ori_x(data):
	global RH_ori_x
	RH_ori_x = data.data
	rospy.loginfo("RH ori x : %f", RH_ori_x)

def call_RH_ori_y(data):
	global RH_ori_y
	RH_ori_y = data.data
	rospy.loginfo("RH ori y : %f", RH_ori_y)

def call_RH_ori_z(data):
	global RH_ori_z
	RH_ori_z = data.data
	rospy.loginfo("RH ori z : %f", RH_ori_z)


def create_subs():
	rospy.init_node('subscribers', anonymous=True)

	rospy.Subscriber('LH_pos_x', Float64, call_LH_pos_x)
	rospy.Subscriber('LH_pos_y', Float64, call_LH_pos_y)
	rospy.Subscriber('LH_pos_z', Float64, call_LH_pos_z)
	rospy.Subscriber('LH_ori_w', Float64, call_LH_ori_w)
	rospy.Subscriber('LH_ori_x', Float64, call_LH_ori_x)
	rospy.Subscriber('LH_ori_y', Float64, call_LH_ori_y)
	rospy.Subscriber('LH_ori_z', Float64, call_LH_ori_z)

	rospy.Subscriber('RH_pos_x', Float64, call_RH_pos_x)
	rospy.Subscriber('RH_pos_y', Float64, call_RH_pos_y)
	rospy.Subscriber('RH_pos_z', Float64, call_RH_pos_z)
	rospy.Subscriber('RH_ori_w', Float64, call_RH_ori_w)
	rospy.Subscriber('RH_ori_x', Float64, call_RH_ori_x)
	rospy.Subscriber('RH_ori_y', Float64, call_RH_ori_y)
	rospy.Subscriber('RH_ori_z', Float64, call_RH_ori_z)

	rospy.spin()

if __name__=='__main__':
	create_subs()