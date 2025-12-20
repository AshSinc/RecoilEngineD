/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

#ifndef _PIVOT_CONTROLLER_H
#define _PIVOT_CONTROLLER_H

#include "CameraController.h"
#include "Game/UI/MouseHandler.h"
#include "Game/Camera.h"

class CPivotController : public CCameraController
{
public:
	CPivotController();

	const std::string GetName() const { return "pivot"; }

	void KeyMove(float3 move);
	void MouseMove(float3 move);
	void ScreenEdgeMove(float3 move);

	void MouseWheelMove(float move) { MouseWheelMove(move, mouse->dir); }
	void MouseWheelMove(float move, const float3& newDir);

	void SetPos(const float3& newPos);
	void SetRot(const float3& newRot) { rot = newRot; };

	float3 SwitchFrom() const;
	void SwitchTo(const CCameraController* oldCam, const bool showText);

	void GetState(StateMap& sm) const;
	bool SetState(const StateMap& sm);
	float3 GetRot() const { return rot; }
	float3 GetDir() const {	return CCamera::GetFwdFromRot(rot); }

	void Update();

	void SetPivot(float x, float y, float z) {this->pivot = float3(x,y,z);}; //called with Spring.SetPivotCameraPivotPoint(x, y, z) to update pivot point
	
private:
	float3 rot;

	float CLAMP_PITCH_MIN = math::PI * 0.1;
	float CLAMP_PITCH_MAX = math::PI * 0.9;

	float zoomMin;
	float zoomMax;

	float3 pivot;
	float3 curPivot;

	float pivotChangeSpeed;
	float edgeMoveSpeed;
	float mouseMoveSpeed;

	float targetZoomLevel;
	float curZoomLevel;

	float curAzimuthAngle;
	float curInclinationAngle;

	float3 avel;

	bool rotateBelowPlane;

	float deceleration;
};

#endif // _PIVOT_CONTROLLER_H
