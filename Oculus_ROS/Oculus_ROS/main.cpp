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
#define leftHand 0
#define rightHand 1
char *ROS_MASTER = "130.209.247.100:11411";

using namespace glm;
using std::string;

int _tmain(int argc, _TCHAR * argv[])
{
	// Wait for user to get into position to start tracking
	printf("Commencing 4 second countdown for user to get into position...");
	std::this_thread::sleep_for(std::chrono::milliseconds(4000));
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

	// Recenter tracking origin to current position:
	ovr_RecenterTrackingOrigin(session);

	// Create tracking states and robot state
	ovrTrackingState state;
	ovrInputState inputState;	// Controller buttons
	ros::NodeHandle nh;
	printf("Connecting to server at %s\n", ROS_MASTER);
	nh.initNode(ROS_MASTER);

	RobotState robot(nh);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	// In a loop, query the HMD for ts current tracking state, then publish it to the robot.
	while (true) {

		state = ovr_GetTrackingState(session, ovr_GetTimeInSeconds(), ovrTrue);
		if (state.StatusFlags & (ovrStatus_OrientationTracked | ovrStatus_PositionTracked))
		{
			// --------- Touch controllers (left hand LH and right hand RH) position, orientation, euler conversion ----------
			if (state.HandStatusFlags[leftHand] == ON_STATUS && state.HandStatusFlags[rightHand] == ON_STATUS)
				printf("Touch controllers are tracked.\n");
			else printf("Touch controllers are NOT tracked. Please move them in range. \n\n");
			printf("Touch controller first status bit:%d and second status bit: %d\n", state.HandStatusFlags[0], state.HandStatusFlags[1]);

			ovrVector3f LHpos = state.HandPoses[leftHand].ThePose.Position;
			printf("\nLeft hand position is: %.2f, %.2f, %.2f\n", LHpos.x, LHpos.y, LHpos.z);

			ovrQuatf LHorient = state.HandPoses[leftHand].ThePose.Orientation;
			quat LHorient_quat = quat(LHorient.w, LHorient.x, LHorient.y, LHorient.z); // convert to glm quaternion for printing
			printf("Left hand quaternion is: ");
			std::cout << glm::to_string(LHorient_quat) << std::endl;

			vec3 LHorient_eul = eulerAngles(LHorient_quat);
			printf("The equivalent LH euler angles are:\nPitch(x): %.3f\nYaw(y): %.3f\nRoll(z): %3f\n", 
				RADTODEG(LHorient_eul.x), RADTODEG(LHorient_eul.y), RADTODEG(LHorient_eul.z));

			ovrVector3f RHpos = state.HandPoses[rightHand].ThePose.Position;
			printf("\nRight hand position is: %.2f, %.2f, %.2f\n", RHpos.x, RHpos.y, RHpos.z);

			ovrQuatf RHorient = state.HandPoses[rightHand].ThePose.Orientation;
			quat RHorient_quat = quat(RHorient.w, RHorient.x, RHorient.y, RHorient.z);  // convert to glm quaternion for printing
			printf("Right hand quaternion is: ");
			std::cout << glm::to_string(RHorient_quat) << std::endl;

			vec3 RHorient_eul = eulerAngles(RHorient_quat);
			printf("The equivalent RH euler angles are:\nPitch(x): %.3f\nYaw(y): %.3f\nRoll(z): %3f\n", 
				RADTODEG(RHorient_eul.x), RADTODEG(RHorient_eul.y), RADTODEG(RHorient_eul.z));

			// Update RobotState and publish
			robot.LH_pos_x_msg.data = LHpos.x;
			robot.LH_pos_y_msg.data = LHpos.y;
			robot.LH_pos_z_msg.data = LHpos.z;

			robot.LH_roll_msg.data = LHorient_eul.z;
			robot.LH_pitch_msg.data = LHorient_eul.x;
			robot.LH_yaw_msg.data = LHorient_eul.y;

			robot.RH_pos_x_msg.data = RHpos.x;
			robot.RH_pos_y_msg.data = RHpos.y;
			robot.RH_pos_z_msg.data = RHpos.z;

			robot.RH_roll_msg.data = RHorient_eul.z;
			robot.RH_pitch_msg.data = RHorient_eul.x;
			robot.RH_yaw_msg.data = RHorient_eul.y;

			robot.publishPose();

			// Button controls
			if (OVR_SUCCESS(ovr_GetInputState(session, ovrControllerType_Touch, &inputState)))
			{
				// Publish robot pose reset if right thumbstick pressed
				if (inputState.Buttons & ovrButton_RThumb)
				{
					robot.reset_pose_msg.data = "human";
					robot.resetPose();
				}
				if (inputState.Buttons & ovrButton_LThumb)
				{
					robot.reset_pose_msg.data = "baxter";
					robot.resetPose();
				}
				// Publish index trigger to grip objects with the arms
				if (inputState.IndexTrigger[rightHand] > 0.01f)
				{
					robot.right_grip_msg.data = inputState.IndexTrigger[rightHand];
					robot.rightGrip();
				} 
				else
				{
					robot.right_grip_msg.data = 0.0;
					robot.rightGrip();
				}
				if (inputState.IndexTriggerNoDeadzone[leftHand] > 0.01f)
				{
					robot.left_grip_msg.data = inputState.IndexTrigger[leftHand];
					robot.leftGrip();
				}
				else
				{
					robot.left_grip_msg.data = 0.0;
					robot.leftGrip();
				}
				// Activate wrist roll mode for each hand
				if (inputState.Buttons & ovrButton_A)
				{
					robot.RH_mode_roll_msg.data = 1.0;
					robot.RHModeRoll();
				}
				else
				{
					robot.RH_mode_roll_msg.data = 0.0;
					robot.RHModeRoll();
				}
				if (inputState.Buttons & ovrButton_X)
				{
					robot.LH_mode_roll_msg.data = 1.0;
					robot.LHModeRoll();
				}
				else
				{
					robot.LH_mode_roll_msg.data = 0.0;
					robot.LHModeRoll();
				}
			}

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

			// Publish head pan (yaw)
			robot.head_pan_msg.data = euler.y;
			robot.headPan();

			// ---------- Clear screen ----------
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