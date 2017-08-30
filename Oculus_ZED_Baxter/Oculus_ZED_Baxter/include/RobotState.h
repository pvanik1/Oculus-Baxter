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
	
	void publishPose();
	void resetPose();
	void leftGrip();
	void rightGrip();
	void headPan();
	void RHModeRoll();
	void LHModeRoll();

	ros::NodeHandle mynh;

	std_msgs::Float64 LH_mode_roll_msg;
	ros::Publisher LH_mode_roll_pub{ "LH_mode_roll", &LH_mode_roll_msg };

	std_msgs::Float64 RH_mode_roll_msg;
	ros::Publisher RH_mode_roll_pub{ "RH_mode_roll", &RH_mode_roll_msg };

	std_msgs::Float64 RH_roll_msg;
	ros::Publisher RH_roll_pub{ "RH_roll", &RH_roll_msg };
	std_msgs::Float64 RH_pitch_msg;
	ros::Publisher RH_pitch_pub{ "RH_pitch", &RH_pitch_msg };
	std_msgs::Float64 RH_yaw_msg;
	ros::Publisher RH_yaw_pub{ "RH_yaw", &RH_yaw_msg };

	std_msgs::Float64 LH_roll_msg;
	ros::Publisher LH_roll_pub{ "LH_roll", &RH_roll_msg };
	std_msgs::Float64 LH_pitch_msg;
	ros::Publisher LH_pitch_pub{ "LH_pitch", &LH_pitch_msg };
	std_msgs::Float64 LH_yaw_msg;
	ros::Publisher LH_yaw_pub{ "LH_yaw", &LH_yaw_msg };

	std_msgs::Float64 head_pan_msg;
	ros::Publisher head_pan_pub{ "head_pan", &head_pan_msg };

	std_msgs::Float64 left_grip_msg;
	ros::Publisher left_grip_pub{ "left_grip", &left_grip_msg };
	std_msgs::Float64 right_grip_msg;
	ros::Publisher right_grip_pub{ "right_grip", &right_grip_msg };

	std_msgs::String reset_pose_msg;
	ros::Publisher reset_pose_pub{ "reset_pose", &reset_pose_msg };

	std_msgs::Float64 LH_pos_x_msg;
	ros::Publisher LH_pos_x_pub{ "LH_pos_x", &LH_pos_x_msg };
	std_msgs::Float64 LH_pos_y_msg;
	ros::Publisher LH_pos_y_pub{ "LH_pos_y", &LH_pos_y_msg };
	std_msgs::Float64 LH_pos_z_msg;
	ros::Publisher LH_pos_z_pub{ "LH_pos_z", &LH_pos_z_msg };

	std_msgs::Float64 RH_pos_x_msg;
	ros::Publisher RH_pos_x_pub{ "RH_pos_x", &RH_pos_x_msg };
	std_msgs::Float64 RH_pos_y_msg;
	ros::Publisher RH_pos_y_pub{ "RH_pos_y", &RH_pos_y_msg };
	std_msgs::Float64 RH_pos_z_msg;
	ros::Publisher RH_pos_z_pub{ "RH_pos_z", &RH_pos_z_msg };
};
