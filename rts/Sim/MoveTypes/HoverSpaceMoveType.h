/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

#ifndef C_SPACE_MOVE_TYPE_H
#define C_SPACE_MOVE_TYPE_H

#include "ASpaceMoveType.h"

struct float4;

class HoverSpaceMoveType: public ASpaceMoveType
{
	CR_DECLARE_DERIVED(HoverSpaceMoveType)
public:
	HoverSpaceMoveType(CUnit* owner);

	void* GetPreallocContainer() { return owner; }  // creg

	// MoveType interface
	bool Update() override;
	void SlowUpdate() override;

	void StartMoving(float3 pos, float goalRadius) override;
	void StartMoving(float3 pos, float goalRadius, float speed) override;
	void KeepPointingTo(float3 pos, float distance, bool aggressive) override; //Can't remove this yet, overrides from abstract further up
	void StopMoving(bool callScript = false, bool hardStop = false, bool cancelRaw = false) override;

	bool SetMemberValue(unsigned int memberHash, void* memberValue) override;

	void ForceHeading(short h);
	void SetGoal(const float3& pos, float distance = 0.0f) override;
	void SetState(SpacecraftMovementState newState) override;
	
	// Main state handlers
	void UpdateTakeoff();
	void UpdateFlying();
	void UpdateHovering();

	float GetGoalRadius(float s = 0.0f) const override { return (SQUARE_SIZE * SQUARE_SIZE); }

	short GetWantedHeading() const { return wantedHeading; }
	short GetForcedHeading() const { return forcedHeading; }

private:
	// Helpers for (multiple) state handlers
	void UpdateHeading();
	void UpdateBanking(bool noBanking);
	bool UpdateAirPhysics();
	void UpdateMoveRate();

	void ExecuteStop();
	void Takeoff() override;
	//void Land() override;

	bool HandleCollisions(bool checkCollisions);

public:
	enum FlyState {
		FLY_CRUISING,
		FLY_CIRCLING,
		FLY_ATTACKING,
		FLY_LANDING
	} flyState;

	bool airStrafe;
	/// Set to true on StopMove, to be able to not stop if a new order comes directly after
	bool wantToStop;

	/// Used when circling something
	float goalDistance;

	
	

	float turnRate;

	float maxDrift;
	float maxTurnAngle;

private:
	float3 wantedSpeed;
	/// Used to determine banking (since it is the current acceleration)
	float3 deltaSpeed;

	float3 circlingPos;
	/// buffets the plane when idling
	float3 randomWind;

	/// force the aircraft to turn toward specific heading (for transports)
	bool forceHeading;

	/// TODO: Seems odd to use heading in unit, since we have toggled useHeading to false..
	short wantedHeading;
	short forcedHeading;

	/// need to pause between circling steps
	int waitCounter;
	/// Scripts expect moverate functions to be called
	int lastMoveRate;
};

#endif // TA_AIR_MOVE_TYPE_H
