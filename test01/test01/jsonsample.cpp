#include "stdafx.h"
#include "json.hpp"
#include <iostream>
#include "cbor.h"

using json = nlohmann::json;
using namespace std;


void create_JSON() {
	json j = {
		{ "position",
		{
			{ "x", 0.3 },
			{ "y", 0.4 },
			{ "z", 0.5 }
		}
		},
		{ "orientation",
		{
			{ "w", 0.01 },
			{ "x", 0.02 },
			{ "y", 0.03 },
			{ "z",  0.04 }
		} }
	};

	json j2 = R"({
	"position":{
		"x":0.3,
		"y":0.4,
		"z":0.5
		},
	"orientation":{
		"w":0.01,
		"x":0.02,
		"y":0.03,
		"z":0.04
		}
	})"_json;


	std::vector<std::uint8_t> v_cbor = json::to_cbor(j);
	json j3 = json::from_cbor(v_cbor);

	 if ((j == j2) && (j2 == j3) && (j == j3)) { cout << "Is correct!\n"; }
	 else { cout << "Is not correct\n"; };

}

//geometry_msgs::Twist twist_msg;
//ros::Publisher cmd_vel_pub("cmd_vel", &twist_msg);
//nh.advertise(cmd_vel_pub);
//
//printf("go robot go!");
//while (1) {
//	twist_msg.linear.x = 5.1;
//	twist_msg.linear.y = 0;
//	twist_msg.linear.z = 0;
//	twist_msg.angular.x = 0;
//	twist_msg.angular.x = 0;
//	twist_msg.angular.x = -1.8;
//
//	cmd_vel_pub.publish(&twist_msg);
//
//	nh.spinOnce();
//	Sleep(100);
//}