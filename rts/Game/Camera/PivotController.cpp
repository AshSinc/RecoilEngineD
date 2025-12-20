/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */


#include "PivotController.h"

#include "Game/Camera.h"
#include "Map/ReadMap.h"
#include "Rendering/GlobalRendering.h"
#include "System/Config/ConfigHandler.h"
#include "System/Log/ILog.h"
#include "Game/CameraHandler.h"

#include "System/Misc/TracyDefs.h"

CONFIG(float, PivotCamScrollSpeed).defaultValue(0.20f);
CONFIG(bool, PivotCamEnabled).defaultValue(true).headlessValue(false);
CONFIG(float, PivotCamFOV).defaultValue(80.0f);
CONFIG(bool, PivotCamRotateBelowPlane).defaultValue(true);
CONFIG(float, PivotCamDeceleration).defaultValue(0.95f);
CONFIG(float, PivotCamChangeSpeed).defaultValue(0.005f);
CONFIG(float, PivotCamEdgeMoveSpeed).defaultValue(1.0f);
CONFIG(float, PivotCamMouseMoveSpeed).defaultValue(1.0f);
CONFIG(float, PivotCamZoomMin).defaultValue(200.0f);
CONFIG(float, PivotCamZoomMax).defaultValue(20000.0f);

CPivotController::CPivotController()
{
	RECOIL_DETAILED_TRACY_ZONE;
	scrollSpeed = configHandler->GetFloat("PivotCamScrollSpeed");
	enabled     = configHandler->GetBool("PivotCamEnabled");
	fov         = configHandler->GetFloat("PivotCamFOV");
	rotateBelowPlane = configHandler->GetFloat("PivotCamRotateBelowPlane");
	deceleration = configHandler->GetFloat("PivotCamDeceleration");
	pivotChangeSpeed = configHandler->GetFloat("PivotCamChangeSpeed");
	edgeMoveSpeed = configHandler->GetFloat("PivotCamEdgeMoveSpeed");
	mouseMoveSpeed = configHandler->GetFloat("PivotCamMouseMoveSpeed");
	zoomMin = configHandler->GetFloat("PivotCamZoomMin");
	zoomMax = configHandler->GetFloat("PivotCamZoomMax");

	//init
	curAzimuthAngle = 0;
	curInclinationAngle = 90 * math::PI / 180;
	targetZoomLevel = 0;
	curZoomLevel = zoomMax/2;
	curPivot = float3(0,0,0);
	if(!rotateBelowPlane)
		CLAMP_PITCH_MIN = math::PI/2;
}

void CPivotController::KeyMove(float3 move)
{
	RECOIL_DETAILED_TRACY_ZONE;

	avel.y += move.y * math::PI / 180;
	avel.x += move.x * math::PI / 180;
}

void CPivotController::MouseMove(float3 move)
{
	RECOIL_DETAILED_TRACY_ZONE;
	KeyMove(move * 0.001f * mouseMoveSpeed);
}

void CPivotController::ScreenEdgeMove(float3 move)
{
	RECOIL_DETAILED_TRACY_ZONE;
	KeyMove(move * edgeMoveSpeed);
}

void CPivotController::MouseWheelMove(float move, const float3& newDir)
{
	RECOIL_DETAILED_TRACY_ZONE;
	targetZoomLevel += move * scrollSpeed;
}

void CPivotController::Update()
{
	RECOIL_DETAILED_TRACY_ZONE;
	const float ft = globalRendering->lastFrameTime;

	//if pivot has changed slowly update current pivot
	if (curPivot != pivot)
		curPivot += (pivot - curPivot) * pivotChangeSpeed * ft;

	//smooth zoom level
	float radius = curZoomLevel + (targetZoomLevel * ft);
	radius = std::clamp(radius, zoomMin, zoomMax);

	//smooth azimuth and inclination angles
	float azimuth = (curAzimuthAngle + (avel.x * ft));
	float inclination = (curInclinationAngle + (avel.y * ft));
	inclination = std::clamp(inclination, CLAMP_PITCH_MIN, CLAMP_PITCH_MAX);

	//keepp azimuth in range of 0 to 2*PI
	if(azimuth > 2*math::PI)
		azimuth -= 2*math::PI;
	else if (azimuth < 0)
		azimuth += 2*math::PI;

	//calculate point on sphere given azimuth and inclination 
	float x = radius * sin(inclination) * cos(azimuth);
	float z = radius * sin(inclination) * sin(azimuth);
	float y = radius * cos(inclination);

	//update postion and rotation
	pos = float3(x,y,z) + curPivot;
	camera->SetDir((curPivot - pos).SafeANormalize());
	rot = camera->GetRot();
	dir = camera->GetDir();

	//decelerate motion for next loop
	avel *= deceleration;
	targetZoomLevel *= deceleration;

	//update vars
	curInclinationAngle = inclination;
	curAzimuthAngle = azimuth;
	curZoomLevel = radius;
}

void CPivotController::SetPos(const float3& newPos)
{
	RECOIL_DETAILED_TRACY_ZONE;
	CCameraController::SetPos(newPos);
	Update();
}


float3 CPivotController::SwitchFrom() const
{
	RECOIL_DETAILED_TRACY_ZONE;
	return pos;
}


void CPivotController::SwitchTo(const CCameraController* oldCam, const bool showText)
{
	RECOIL_DETAILED_TRACY_ZONE;
	if (showText) {
		LOG("Switching to Pivot camera");
	}
	float3 newPos = oldCam->SwitchFrom();
	if (oldCam->GetName() == "ov") {
		pos = float3(newPos.x, pos.y, newPos.z);
		Update();
		return;
	}
	pos = newPos;
	rot = oldCam->GetRot();
}

void CPivotController::GetState(StateMap& sm) const
{
	RECOIL_DETAILED_TRACY_ZONE;
	sm["zoom"] = fov;

	sm["px"] = pos.x;
	sm["py"] = pos.y;
	sm["pz"] = pos.z;

	sm["dx"] = dir.x;
	sm["dy"] = dir.y;
	sm["dz"] = dir.z;
}


bool CPivotController::SetState(const StateMap& sm)
{
	RECOIL_DETAILED_TRACY_ZONE;
	SetStateFloat(sm, "fov", fov);

	SetStateFloat(sm, "px", pos.x);
	SetStateFloat(sm, "py", pos.y);
	SetStateFloat(sm, "pz", pos.z);

	SetStateFloat(sm, "dx", dir.x);
	SetStateFloat(sm, "dy", dir.y);
	SetStateFloat(sm, "dz", dir.z);

	return true;
}