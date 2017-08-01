#pragma once

#include <iostream>

#include "ros.h"
#include "WindowsSocket.h"
#include <std_msgs\String.h>
#include <std_msgs\Float64.h>

class RobotState
{
public:
	RobotState(ros::NodeHandle nh);
	~RobotState();
	//void publish(ros::NodeHandle nh);
	void publish();

	ros::NodeHandle mynh;

	std_msgs::Float64 LH_pos_x_msg;
	ros::Publisher LH_pos_x_pub{ "LH_pos_x", &LH_pos_x_msg };
	std_msgs::Float64 LH_pos_y_msg;
	ros::Publisher LH_pos_y_pub{ "LH_pos_y", &LH_pos_y_msg };
	std_msgs::Float64 LH_pos_z_msg;
	ros::Publisher LH_pos_z_pub{ "LH_pos_z", &LH_pos_z_msg };
	std_msgs::Float64 LH_ori_w_msg;
	ros::Publisher LH_ori_w_pub{ "LH_ori_w", &LH_ori_w_msg };
	std_msgs::Float64 LH_ori_x_msg;
	ros::Publisher LH_ori_x_pub{ "LH_ori_x", &LH_ori_x_msg };
	std_msgs::Float64 LH_ori_y_msg;
	ros::Publisher LH_ori_y_pub{ "LH_ori_y", &LH_ori_y_msg };
	std_msgs::Float64 LH_ori_z_msg;
	ros::Publisher LH_ori_z_pub{ "LH_ori_z", &LH_ori_z_msg };

	std_msgs::Float64 RH_pos_x_msg;
	ros::Publisher RH_pos_x_pub{ "RH_pos_x", &RH_pos_x_msg };
	std_msgs::Float64 RH_pos_y_msg;
	ros::Publisher RH_pos_y_pub{ "RH_pos_y", &RH_pos_y_msg };
	std_msgs::Float64 RH_pos_z_msg;
	ros::Publisher RH_pos_z_pub{ "RH_pos_z", &RH_pos_z_msg };
	std_msgs::Float64 RH_ori_w_msg;
	ros::Publisher RH_ori_w_pub{ "RH_ori_w", &RH_ori_w_msg };
	std_msgs::Float64 RH_ori_x_msg;
	ros::Publisher RH_ori_x_pub{ "RH_ori_x", &RH_ori_x_msg };
	std_msgs::Float64 RH_ori_y_msg;
	ros::Publisher RH_ori_y_pub{ "RH_ori_y", &RH_ori_y_msg };
	std_msgs::Float64 RH_ori_z_msg;
	ros::Publisher RH_ori_z_pub{ "RH_ori_z", &RH_ori_z_msg };

};
