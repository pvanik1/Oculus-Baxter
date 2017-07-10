// myoculus2.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <OVR_CAPI.h>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <stdio.h>
#include <math.h>
#include "glm/ext.hpp"
#include "glm/gtx/string_cast.hpp"


#define PI 3.14159265358979323846264338327950
#define RADTODEG(x) ( (x) * 180.0 / PI )
#define DEGTORAD(x) ( (x) * PI / 180.0 )

using namespace glm;

void Application()
{
	ovrResult result = ovr_Initialize(nullptr);
	if (OVR_FAILURE(result))
		return;
	ovrSession session;
	ovrGraphicsLuid luid;
	result = ovr_Create(&session, &luid);
	if (OVR_FAILURE(result))
	{
		ovr_Shutdown();
		printf("we shut down...\n");
		return;
	}
	ovrHmdDesc desc = ovr_GetHmdDesc(session);

	ovrSizei resolution = desc.Resolution;
	printf("The HMD resolution is %d x %d pixels.\n", resolution.w, resolution.h);


	// Query the HMD for ts current tracking state.
	ovrTrackingState state = ovr_GetTrackingState(session, ovr_GetTimeInSeconds(), ovrTrue);
	if (state.StatusFlags & (ovrStatus_OrientationTracked | ovrStatus_PositionTracked))
	{
		ovrPosef pose = state.HeadPose.ThePose;

		ovrQuatf o = pose.Orientation;

		ovrVector3f p = pose.Position;
		printf("\nPosition is: %.2f, %.2f, %.2f\n", p.x, p.y, p.z);


		//example of creating quaternion from euler angles
		//vec3 EulerAngles( 1.5, 1.5, 0);
		//quat MyQuaternion = quat(EulerAngles);

		// this converts from ovrQuatf to glm quat
		quat myQuat = quat(o.w, o.x, o.y, o.z);
		printf("myQuat from sensor is: ");
		std::cout << glm::to_string(myQuat) << std::endl;

		vec3 euler = eulerAngles(myQuat);// Returns euler angles, pitch as x, yaw as y, roll as z.
		printf("The euler angles are:\nPitch(x): %.3f\nYaw(y): %.3f\nRoll(z): %3f\n", RADTODEG(euler.x), RADTODEG(euler.y), RADTODEG(euler.z));

		printf("------TEST-----\n");
		quat testq = quat(cos(45*PI/180),0,sin(45 * PI / 180),0);
		printf("Here is my quaternion from the calculated cos and sin values:\n");
		std::cout << glm::to_string(testq) << std::endl;

		vec3 eulertest(0, 1.5708, 0);
		quat eulerquat = quat(eulertest);
		printf("Here is the euler test quaternion: \n");
		std::cout << glm::to_string(eulerquat) << std::endl;

		vec3 original = eulerAngles(testq);
		original.x *= 180.0 / PI;
		original.y *= 180.0 / PI;
		original.z *= 180.0 / PI;
		printf("Here is the degree converted quaternion:\n");
		std::cout << glm::to_string(original) << std::endl;


	}
	ovr_Destroy(session);
	ovr_Shutdown();
}

int main()
{
	printf("starting! :)\n");
	Application();
	printf("finished! :)\n");
    return 0;
	
}
