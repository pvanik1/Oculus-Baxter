#include "RobotState.h"

RobotState::RobotState(ros::NodeHandle nh)
{
	mynh = nh;

	// Advertise publishers. 
	printf("Advertising button publishers");
	mynh.advertise(reset_pose_pub);
	mynh.advertise(left_grip_pub);
	mynh.advertise(right_grip_pub);

	printf("Advertising LH publishers...\n");
	mynh.advertise(LH_pos_x_pub);
	mynh.advertise(LH_pos_y_pub);
	mynh.advertise(LH_pos_z_pub);
	mynh.advertise(LH_ori_w_pub);
	mynh.advertise(LH_ori_x_pub);
	mynh.advertise(LH_ori_y_pub);
	mynh.advertise(LH_ori_z_pub);
 
	printf("Advertising RH publishers...\n\n");
	mynh.advertise(RH_pos_x_pub);
	mynh.advertise(RH_pos_y_pub);
	mynh.advertise(RH_pos_z_pub);
	mynh.advertise(RH_ori_w_pub);
	mynh.advertise(RH_ori_x_pub);
	mynh.advertise(RH_ori_y_pub);
	mynh.advertise(RH_ori_z_pub);
}

RobotState::~RobotState()
{
}

// ROS publishers publishing message data
void RobotState::publishPose() {
	printf("\n\nPublishing...\n\n");
	//printf("data: %f\n---\n", LH_pos_x_msg.data);
	LH_pos_x_pub.publish(&LH_pos_x_msg);	
	LH_pos_y_pub.publish(&LH_pos_y_msg);	
	LH_pos_z_pub.publish(&LH_pos_z_msg);	
	LH_ori_w_pub.publish(&LH_ori_w_msg);	
	LH_ori_x_pub.publish(&LH_ori_x_msg);	
	LH_ori_y_pub.publish(&LH_ori_y_msg);	
	LH_ori_z_pub.publish(&LH_ori_z_msg);
	
	RH_pos_x_pub.publish(&RH_pos_x_msg);	
	RH_pos_y_pub.publish(&RH_pos_y_msg);	
	RH_pos_z_pub.publish(&RH_pos_z_msg);
	RH_ori_w_pub.publish(&RH_ori_w_msg);
	RH_ori_x_pub.publish(&RH_ori_x_msg);
	RH_ori_y_pub.publish(&RH_ori_y_msg);	
	RH_ori_z_pub.publish(&RH_ori_z_msg);

	mynh.spinOnce();
	printf("Published\n\n");
}

void RobotState::resetPose() {
	reset_pose_pub.publish(&reset_pose_msg);
	mynh.spinOnce();
	printf("Pose reset");
}

void RobotState::leftGrip() {
	left_grip_pub.publish(&left_grip_msg);
	mynh.spinOnce();
	printf("Left grip published");
}

void RobotState::rightGrip() {
	right_grip_pub.publish(&right_grip_msg);
	mynh.spinOnce();
	printf("Right grip published");
}
