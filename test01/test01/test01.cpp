#include "json.hpp"
#include <iostream>

#include "tchar.h"
#include "ros.h"
#include "duration.cpp"
#include "time.cpp"
#include "WindowsSocket.h"
#include "WindowsSocket.cpp"
#include <std_msgs\String.h>
#include <std_msgs\Float64.h>

using std::string;

int _tmain(int argc, _TCHAR * argv[])
{
	// Connect to ROS Master
	ros::NodeHandle nh;
	char *ros_master = "130.209.247.100:11411";
	printf("Connecting to server at %s\n", ros_master);
	nh.initNode(ros_master);

	// Construct Position and Orientation (quaternion) messages and publishers. 
	// Advertise them and give default values.
	printf("Constructing messages...\n");

	std_msgs::Float64 pos_x_msg;
	ros::Publisher pos_x_pub("pos_x", &pos_x_msg);
	nh.advertise(pos_x_pub);
	pos_x_msg.data = 1.0;

	std_msgs::Float64 pos_y_msg;
	ros::Publisher pos_y_pub("pos_y", &pos_y_msg);
	nh.advertise(pos_y_pub);
	pos_y_msg.data = 22.0;

	std_msgs::Float64 pos_z_msg;
	ros::Publisher pos_z_pub("pos_z", &pos_z_msg);
	nh.advertise(pos_z_pub);
	pos_z_msg.data = 333.0;

	std_msgs::Float64 ori_w_msg;
	ros::Publisher ori_w_pub("ori_w", &ori_w_msg);
	nh.advertise(ori_w_pub);
	ori_w_msg.data = 4444.0;

	std_msgs::Float64 ori_x_msg;
	ros::Publisher ori_x_pub("ori_x", &ori_x_msg);
	nh.advertise(ori_x_pub);
	ori_x_msg.data = 55555.0;

	std_msgs::Float64 ori_y_msg;
	ros::Publisher ori_y_pub("ori_y", &ori_y_msg);
	nh.advertise(ori_y_pub);
	ori_y_msg.data = 666666.0;

	std_msgs::Float64 ori_z_msg;
	ros::Publisher ori_z_pub("ori_z", &ori_z_msg);
	nh.advertise(ori_z_pub);
	ori_z_msg.data = 7777777.0;

	printf("Go robot go!\n");
	for (int i=0; i<5; i++)
	{
		
		printf("data: %f\n---\n", pos_x_msg.data);
		pos_x_pub.publish(&pos_x_msg);
		nh.spinOnce();
		Sleep(50);

		printf("data: %f\n---\n", pos_y_msg.data);
		pos_y_pub.publish(&pos_y_msg);
		nh.spinOnce();
		Sleep(50);

		printf("data: %f\n---\n", pos_z_msg.data);
		pos_z_pub.publish(&pos_z_msg);
		nh.spinOnce();
		Sleep(50);

		printf("data: %f\n---\n", ori_w_msg.data);
		ori_w_pub.publish(&ori_w_msg);
		nh.spinOnce();
		Sleep(50);

		printf("data: %f\n---\n", ori_x_msg.data);
		ori_x_pub.publish(&ori_x_msg);
		nh.spinOnce();
		Sleep(50);

		printf("data: %f\n---\n", ori_y_msg.data);
		ori_y_pub.publish(&ori_y_msg);
		nh.spinOnce();
		Sleep(50);

		printf("data: %f\n---\n", ori_z_msg.data);
		ori_z_pub.publish(&ori_z_msg);
		nh.spinOnce();
		Sleep(50);

		pos_x_msg.data += 1.0;
		pos_y_msg.data += 1.0;
		pos_z_msg.data += 1.0;
		ori_w_msg.data += 1.0;
		ori_x_msg.data += 1.0;
		ori_y_msg.data += 1.0;
		ori_z_msg.data += 1.0;
	}
	printf("All done!\n");
	return 0;
}