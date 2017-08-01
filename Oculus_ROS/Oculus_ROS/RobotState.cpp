#include "RobotState.h"

RobotState::RobotState(ros::NodeHandle nh)
{
	mynh = nh;

	// Advertise LH and RH position and orientation (quaternions). 
	printf("Advertising LH messages...\n");
	mynh.advertise(LH_pos_x_pub);
	mynh.advertise(LH_pos_y_pub);
	mynh.advertise(LH_pos_z_pub);
	mynh.advertise(LH_ori_w_pub);
	mynh.advertise(LH_ori_x_pub);
	mynh.advertise(LH_ori_y_pub);
	mynh.advertise(LH_ori_z_pub);
 
	printf("Advertising RH messages...\n\n");
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

// ROS publishers publish the message data
void RobotState::publish() {
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
	// Sleep(100); // not necessary because main loop already sleeps for 100?
}
