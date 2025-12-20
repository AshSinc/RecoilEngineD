/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */


#include "PivotController.h"

#include "Game/Camera.h"
#include "Map/ReadMap.h"
#include "Rendering/GlobalRendering.h"
#include "System/SpringMath.h"
#include "System/Config/ConfigHandler.h"
#include "System/Log/ILog.h"
#include "Game/CameraHandler.h"

#include "System/Misc/TracyDefs.h"

CONFIG(float, RotOverheadMouseScale).defaultValue(0.01f);
CONFIG(int, RotOverheadScrollSpeed).defaultValue(1);
CONFIG(bool, RotOverheadEnabled).defaultValue(true).headlessValue(false);
CONFIG(float, RotOverheadFOV).defaultValue(45.0f);
CONFIG(bool, RotOverheadClampMap).defaultValue(true).headlessValue(true);


CPivotController::CPivotController(): rot(2.677f, 0.0f, 0.0f)
{
	// TODO - Need to integrate with existing config vars or create new ones if needed
	RECOIL_DETAILED_TRACY_ZONE;
	// mouseScale  = configHandler->GetFloat("RotOverheadMouseScale");
	// scrollSpeed = configHandler->GetInt("RotOverheadScrollSpeed") * 0.1f;
	// enabled     = configHandler->GetBool("RotOverheadEnabled");
	// fov         = configHandler->GetFloat("RotOverheadFOV");
	// clampToMap = configHandler->GetBool("RotOverheadClampMap");
	scrollSpeed = 10;
	enabled     = true;
	fov         = 80;
	curAzimuthAngle = 0;
	curInclinationAngle = 90 * math::PI / 180;
	targetZoomLevel = 0;
	curZoomLevel = CLAMP_ZOOM_MAX/2;
	curPivot = float3(0,0,0);
}

void CPivotController::KeyMove(float3 move)
{
	RECOIL_DETAILED_TRACY_ZONE;

	const float qy = (move.y == 0.0f) ? 0.0f : (move.y > 0.0f ? 1.0f : -1.0f);
	const float qx = (move.x == 0.0f) ? 0.0f : (move.x > 0.0f ? 1.0f : -1.0f);

	float ZoomDistance = 1;
	const float speed  = 1 * ZoomDistance;
	const float aspeed = 1 * ZoomDistance;

	avel.y += move.y * math::PI / 180;
	avel.x += move.x * math::PI / 180;
}

void CPivotController::MouseMove(float3 move)
{
	RECOIL_DETAILED_TRACY_ZONE;
	KeyMove(move * 0.001f);
}


void CPivotController::ScreenEdgeMove(float3 move)
{
	RECOIL_DETAILED_TRACY_ZONE;
	KeyMove(move);
}


void CPivotController::MouseWheelMove(float move, const float3& newDir)
{
	RECOIL_DETAILED_TRACY_ZONE;
	targetZoomLevel += move * 0.25f;
}

void CPivotController::Update()
{
	RECOIL_DETAILED_TRACY_ZONE;
	const float ft = globalRendering->lastFrameTime;

	if (curPivot != pivot)
	{
		curPivot += (pivot - curPivot) * PIVOT_CHANGE_SPEED *ft;
	}

	float scale = 1;
	float r = curZoomLevel + (targetZoomLevel * ft);
	r = std::clamp(r, CLAMP_ZOOM_MIN, CLAMP_ZOOM_MAX);

	float s = (curAzimuthAngle + (avel.x * ft * scale));
	float t = (curInclinationAngle + (avel.y * ft * scale));
	t = std::clamp(t, CLAMP_PITCH_MIN, CLAMP_PITCH_MAX);

	if(t > math::PI)
	{
		t = math::PI;
	}
	else if (t < 0) {
		t = 0;
	}
	if(s > 2*math::PI)
	{
		s -= 2*math::PI;
	}
	else if (s < 0) {
		s += 2*math::PI;
	}

	float x= r * sin(t) * cos(s);
	float z= r * sin(t) * sin(s);
	float y= r * cos(t);

	pos = float3(x,y,z) + curPivot;

	camera->SetDir((curPivot - pos).SafeANormalize());
	rot = camera->GetRot();
	dir = camera->GetDir();

	float AngularDecel = 0.95;

	avel *= AngularDecel;
	targetZoomLevel *= AngularDecel;

	curInclinationAngle = t;
	curAzimuthAngle = s;
	curZoomLevel = r;
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
		LOG("Switching to Rotatable overhead camera");
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

void CPivotController::SetPivot(float x, float y, float z) 
{
	this->pivot = float3(x,y,z);

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