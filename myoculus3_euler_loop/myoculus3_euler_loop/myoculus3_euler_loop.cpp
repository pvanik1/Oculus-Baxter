// myoculus3_euler_loop.cpp : Defines the entry point for the console application.

#include "stdafx.h"
#include <OVR_CAPI.h>
#include <iostream>
#include <stdio.h>
#include <glm/glm.hpp>
#include "glm/gtx/string_cast.hpp"
#include <thread>

#define PI 3.14159265358979323846
#define RADTODEG(x) ( (x) * 180.0 / PI )
#define DEGTORAD(x) ( (x) * PI / 180.0 )
#define ON_STATUS 3

using namespace glm;


void Application()
{
	// Boiler plate
	ovrResult result = ovr_Initialize(nullptr);
	if (OVR_FAILURE(result))
		return;
	ovrSession session;
	ovrGraphicsLuid luid;
	result = ovr_Create(&session, &luid);

	if (OVR_FAILURE(result))
	{
		ovr_Shutdown();
		printf("There was a failure...\n");
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
		return;
	}

	// Recenter tracking origin and yaw to current position:
	ovr_RecenterTrackingOrigin(session);

	// Misc info such as resolution and vendor can be obtained by querying the ovrHmdDesc
	ovrHmdDesc desc = ovr_GetHmdDesc(session);
	ovrSizei resolution = desc.Resolution;
	printf("The HMD resolution is %d x %d pixels.\n", resolution.w, resolution.h);


	// In a loop, query the HMD for ts current tracking state.
	
	ovrTrackingState state;
	ovrInputState inputState;

	while (true) {
		// In a production application, however, you should use
		// the real - time computed value returned by GetPredictedDisplayTime.
		// Prediction is covered in more detail in the section on Frame Timing.

		state = ovr_GetTrackingState(session, ovr_GetTimeInSeconds(), ovrTrue);
		
		if (state.StatusFlags & (ovrStatus_OrientationTracked | ovrStatus_PositionTracked))
		{

			/// --------- Hand position code ------------
			if (state.HandStatusFlags[0] == ON_STATUS && state.HandStatusFlags[1] == ON_STATUS)
				printf("Touch controllers are tracked.\n");
			else printf("Touch controllers are NOT tracked. Please move them in range. \n\n");
			printf("\Touch controller first status bit:%d and second status bit: %d\n", state.HandStatusFlags[0], state.HandStatusFlags[1]);

			// Left hand

			// LH position
			ovrVector3f LHpos = state.HandPoses[0].ThePose.Position;
			printf("\nLeft hand position is: %.2f, %.2f, %.2f\n", LHpos.x, LHpos.y, LHpos.z);

			// LH orientation (quaternion)
			ovrQuatf LHorient = state.HandPoses[0].ThePose.Orientation;
			quat LHorient_quat = quat(LHorient.w, LHorient.x, LHorient.y, LHorient.z);
			printf("Left hand quaternion is: ");
			std::cout << glm::to_string(LHorient_quat) << std::endl;

			// LH orientation (euler)
			vec3 LHorient_eul = eulerAngles(LHorient_quat);
			printf("The equivalent LH euler angles are:\nPitch(x): %.3f\nYaw(y): %.3f\nRoll(z): %3f\n", RADTODEG(LHorient_eul.x), RADTODEG(LHorient_eul.y), RADTODEG(LHorient_eul.z));


			// Right hand

			// RH position
			ovrVector3f RHpos = state.HandPoses[1].ThePose.Position;
			printf("\nRight hand position is: %.2f, %.2f, %.2f\n", RHpos.x, RHpos.y, RHpos.z);

			// RH orientation (quaternion)
			ovrQuatf RHorient = state.HandPoses[1].ThePose.Orientation;
			quat RHorient_quat = quat(RHorient.w, RHorient.x, RHorient.y, RHorient.z);
			printf("Right hand quaternion is: ");
			std::cout << glm::to_string(RHorient_quat) << std::endl;

			// RH orientation (euler)
			vec3 RHorient_eul = eulerAngles(RHorient_quat);
			printf("The equivalent RH euler angles are:\nPitch(x): %.3f\nYaw(y): %.3f\nRoll(z): %3f\n", RADTODEG(RHorient_eul.x), RADTODEG(RHorient_eul.y), RADTODEG(RHorient_eul.z));


			// ---------- Headset Position code ----------
			ovrVector3f p = state.HeadPose.ThePose.Position;
			printf("\nHeadset position is: %.2f, %.2f, %.2f\n", p.x, p.y, p.z);


			// ---------- Headset orientation code ----------
			ovrQuatf o = state.HeadPose.ThePose.Orientation;
			quat myQuat = quat(o.w, o.x, o.y, o.z);
			printf("Headset orientation quaternion is: ");
			std::cout << glm::to_string(myQuat) << std::endl;

			// conversion to Euler angles
			vec3 euler = eulerAngles(myQuat);
			printf("The equivalent euler angles are:\nPitch(x): %.3f\nYaw(y): %.3f\nRoll(z): %3f\n", RADTODEG(euler.x), RADTODEG(euler.y), RADTODEG(euler.z));

			

			// ---------- Wait code ----------
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			std::system("cls");
		}
		else {
			printf("Could not obtain orientation or position. Please check whether the Rift is in range and sensors are operating normally.\n");
		}
	}
	ovr_Destroy(session);
	ovr_Shutdown();
}

int main()
{
	printf("Starting! :)\n");
	Application();
	printf("Finished! :)\n");
	return 0;
}
