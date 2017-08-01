//#include "stdafx.h"

#include <iostream>
#include "tchar.h"
#include <stdio.h>
#include <thread>

#include <OVR_CAPI.h>
#include <glm/glm.hpp>
#include "glm/gtx/string_cast.hpp"

#include "RobotState.h"

#define PI 3.14159265358979323846
#define RADTODEG(x) ( (x) * 180.0 / PI )
#define DEGTORAD(x) ( (x) * PI / 180.0 )
#define ON_STATUS 3

#define HOST "130.209.247.100:11411"
// 130.209.247.100:11411

using namespace glm;
using std::string;

int _tmain(int argc, _TCHAR * argv[])
{
	// Boilerplate
	printf("Initialising Oculus...\n");
	ovrResult result = ovr_Initialize(nullptr);
	if (OVR_FAILURE(result))
		return 0;
	ovrSession session;
	ovrGraphicsLuid luid;
	result = ovr_Create(&session, &luid);

	if (OVR_FAILURE(result))
	{
		ovr_Shutdown();
		printf("OVR_FAILURE - shutting down...\n");
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
		return 0;
	}

	// Recenter tracking origin and yaw to current position:
	ovr_RecenterTrackingOrigin(session);

	// Misc info such as resolution and vendor can be obtained by querying the ovrHmdDesc
	/*ovrHmdDesc desc = ovr_GetHmdDesc(session);
	ovrSizei resolution = desc.Resolution;
	printf("The HMD resolution is %d x %d pixels.\n", resolution.w, resolution.h);*/

	// Create tracking states and robot state
	ovrTrackingState state;
	ovrInputState inputState;	// Controller buttons
	ros::NodeHandle nh;
	char *ros_master = "130.209.247.100:11411";
	printf("Connecting to server at %s\n", ros_master);
	nh.initNode(ros_master);

	RobotState robot(nh);

	// In a loop, query the HMD for ts current tracking state, then publish it to the robot.
	while (true) {

		state = ovr_GetTrackingState(session, ovr_GetTimeInSeconds(), ovrTrue);
		if (state.StatusFlags & (ovrStatus_OrientationTracked | ovrStatus_PositionTracked))
		{
			// --------- Get left hand (LH) and right hand (RH) position and orientation ----------
			if (state.HandStatusFlags[0] == ON_STATUS && state.HandStatusFlags[1] == ON_STATUS)
				printf("Touch controllers are tracked.\n");
			else printf("Touch controllers are NOT tracked. Please move them in range. \n\n");
			printf("Touch controller first status bit:%d and second status bit: %d\n", state.HandStatusFlags[0], state.HandStatusFlags[1]);

			ovrVector3f LHpos = state.HandPoses[0].ThePose.Position;
			printf("\nLeft hand position is: %.2f, %.2f, %.2f\n", LHpos.x, LHpos.y, LHpos.z);

			ovrQuatf LHorient = state.HandPoses[0].ThePose.Orientation;
			quat LHorient_quat = quat(LHorient.w, LHorient.x, LHorient.y, LHorient.z); // convert to glm quaternion for printing
			printf("Left hand quaternion is: ");
			std::cout << glm::to_string(LHorient_quat) << std::endl;

			// LH orientation conversion to euler
			vec3 LHorient_eul = eulerAngles(LHorient_quat);
			printf("The equivalent LH euler angles are:\nPitch(x): %.3f\nYaw(y): %.3f\nRoll(z): %3f\n", 
				RADTODEG(LHorient_eul.x), RADTODEG(LHorient_eul.y), RADTODEG(LHorient_eul.z));

			ovrVector3f RHpos = state.HandPoses[1].ThePose.Position;
			printf("\nRight hand position is: %.2f, %.2f, %.2f\n", RHpos.x, RHpos.y, RHpos.z);

			ovrQuatf RHorient = state.HandPoses[1].ThePose.Orientation;
			quat RHorient_quat = quat(RHorient.w, RHorient.x, RHorient.y, RHorient.z);  // convert to glm quaternion for printing
			printf("Right hand quaternion is: ");
			std::cout << glm::to_string(RHorient_quat) << std::endl;

			// RH orientation conversion to euler
			vec3 RHorient_eul = eulerAngles(RHorient_quat);
			printf("The equivalent RH euler angles are:\nPitch(x): %.3f\nYaw(y): %.3f\nRoll(z): %3f\n", 
				RADTODEG(RHorient_eul.x), RADTODEG(RHorient_eul.y), RADTODEG(RHorient_eul.z));

			// Update RobotState and publish
			robot.LH_pos_x_msg.data = LHpos.x;
			robot.LH_pos_y_msg.data = LHpos.y;
			robot.LH_pos_z_msg.data = LHpos.z;
			robot.LH_ori_w_msg.data = LHorient.w;
			robot.LH_ori_x_msg.data = LHorient.x;
			robot.LH_ori_y_msg.data = LHorient.y;
			robot.LH_ori_z_msg.data = LHorient.z;

			robot.RH_pos_x_msg.data = RHpos.x;
			robot.RH_pos_y_msg.data = RHpos.y;
			robot.RH_pos_z_msg.data = RHpos.z;
			robot.RH_ori_w_msg.data = RHorient.w;
			robot.RH_ori_x_msg.data = RHorient.x;
			robot.RH_ori_y_msg.data = RHorient.y;
			robot.RH_ori_z_msg.data = RHorient.z;

			robot.publish();

			// ---------- Get headset position and orientation ----------
			ovrVector3f p = state.HeadPose.ThePose.Position;
			printf("\nHeadset position is: %.2f, %.2f, %.2f\n", p.x, p.y, p.z);

			ovrQuatf o = state.HeadPose.ThePose.Orientation;
			quat myQuat = quat(o.w, o.x, o.y, o.z);
			printf("Headset orientation quaternion is: ");
			std::cout << glm::to_string(myQuat) << std::endl;

			// Headset orientation conversion to Euler angles
			vec3 euler = eulerAngles(myQuat);
			printf("The equivalent euler angles are:\nPitch(x): %.3f\nYaw(y): %.3f\nRoll(z): %3f\n", RADTODEG(euler.x), RADTODEG(euler.y), RADTODEG(euler.z));


			// ---------- Wait code ----------
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			std::system("cls");
		}
		else {
			printf("Could not obtain orientation or position. Please check whether the Rift is in range and sensors are operating normally.\n");
		}
	}
	ovr_Destroy(session);
	ovr_Shutdown();
	printf("Oculus shut down.\n");
	return 0;
}