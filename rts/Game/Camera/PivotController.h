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

	void SetPivot(float x, float y, float z); //added call from lua

private:
	float3 rot;

	float CLAMP_PITCH_MIN = math::PI * 0.1;
	float CLAMP_PITCH_MAX = math::PI * 0.9;

	float CLAMP_ZOOM_MIN = 100;
	float CLAMP_ZOOM_MAX = 20000;

	float3 pivot;
	float3 curPivot;

	float PIVOT_CHANGE_SPEED = 0.005;

	float targetZoomLevel;
	float curZoomLevel; // current zoom-out distance

	float curAzimuthAngle;
	float curInclinationAngle;

	float3 avel;
};

#endif // _PIVOT_CONTROLLER_H
