#include "RobotState.h"

RobotState::RobotState(ros::NodeHandle nh)
{
	mynh = nh;

	// Advertise publishers. 
	mynh.advertise(LH_mode_roll_pub);
	mynh.advertise(RH_mode_roll_pub);

	mynh.advertise(RH_roll_pub);
	mynh.advertise(RH_pitch_pub);
	mynh.advertise(RH_yaw_pub);

	mynh.advertise(LH_roll_pub);
	mynh.advertise(LH_pitch_pub);
	mynh.advertise(LH_yaw_pub);

	mynh.advertise(head_pan_pub);

	mynh.advertise(reset_pose_pub);
	mynh.advertise(left_grip_pub);
	mynh.advertise(right_grip_pub);

	mynh.advertise(LH_pos_x_pub);
	mynh.advertise(LH_pos_y_pub);
	mynh.advertise(LH_pos_z_pub);
 
	mynh.advertise(RH_pos_x_pub);
	mynh.advertise(RH_pos_y_pub);
	mynh.advertise(RH_pos_z_pub);
}

RobotState::~RobotState()
{
}

// ROS publishers publishing message data
void RobotState::publishPose() {
	RH_roll_pub.publish(&RH_roll_msg);
	RH_pitch_pub.publish(&RH_pitch_msg);
	RH_yaw_pub.publish(&RH_yaw_msg);	
	
	RH_pos_x_pub.publish(&RH_pos_x_msg);	
	RH_pos_y_pub.publish(&RH_pos_y_msg);	
	RH_pos_z_pub.publish(&RH_pos_z_msg);

	LH_pos_x_pub.publish(&LH_pos_x_msg);
	LH_pos_y_pub.publish(&LH_pos_y_msg);
	LH_pos_z_pub.publish(&LH_pos_z_msg);

	LH_roll_pub.publish(&LH_roll_msg);
	LH_pitch_pub.publish(&LH_pitch_msg);
	LH_yaw_pub.publish(&LH_yaw_msg);

	mynh.spinOnce();
}

void RobotState::resetPose() {
	reset_pose_pub.publish(&reset_pose_msg);
	mynh.spinOnce();
	printf("Pose reset");
}

void RobotState::leftGrip() {
	left_grip_pub.publish(&left_grip_msg);
	mynh.spinOnce();
}

void RobotState::rightGrip() {
	right_grip_pub.publish(&right_grip_msg);
	mynh.spinOnce();
}

void RobotState::headPan() {
	head_pan_pub.publish(&head_pan_msg);
	mynh.spinOnce();
}

void RobotState::RHModeRoll() {
	RH_mode_roll_pub.publish(&RH_mode_roll_msg);
	mynh.spinOnce();
}

void RobotState::LHModeRoll() {
	LH_mode_roll_pub.publish(&LH_mode_roll_msg);
}