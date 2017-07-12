#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

# def callback(data):
# 	rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.data)

def call_pos_x(data):
	rospy.loginfo("%f", data.data)

def call_pos_y(data):
	rospy.loginfo("%f", data.data)

def call_pos_z(data):
	rospy.loginfo("%f", data.data)

def call_ori_w(data):
	rospy.loginfo("%f", data.data)

def call_ori_x(data):
	rospy.loginfo("%f", data.data)

def call_ori_y(data):
	rospy.loginfo("%f", data.data)

def call_ori_z(data):
	rospy.loginfo("%f", data.data)


def create_subs():
	rospy.init_node('subscribers', anonymous=True)

	rospy.Subscriber('pos_x', Float64, call_pos_x)
	rospy.Subscriber('pos_y', Float64, call_pos_y)
	rospy.Subscriber('pos_z', Float64, call_pos_z)
	rospy.Subscriber('ori_w', Float64, call_ori_w)
	rospy.Subscriber('ori_x', Float64, call_ori_x)
	rospy.Subscriber('ori_y', Float64, call_ori_y)
	rospy.Subscriber('ori_z', Float64, call_ori_z)

	rospy.spin()

if __name__=='__main__':
	create_subs()