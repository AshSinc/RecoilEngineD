/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

#ifndef A_SPACE_MOVE_TYPE_H_
#define A_SPACE_MOVE_TYPE_H_

#include "MoveType.h"

/**
 * Supposed to be an abstract class.
 * Do not create an instance of this class.
 * Use either CHoverAirMoveType or CStrafeAirMoveType instead.
 */
class ASpaceMoveType : public AMoveType
{
	CR_DECLARE(ASpaceMoveType)
public:
	//typedef float(*GetGroundHeightFunc)(float, float);
	typedef void(*EmitCrashTrailFunc)(CUnit*, unsigned int);

	enum SpacecraftMovementState {
		SPACECRAFT_LANDED,
		SPACECRAFT_FLYING,
		SPACECRAFT_LANDING,
		SPACECRAFT_CRASHING,
		SPACECRAFT_TAKEOFF,
		/// this is what happens to aircraft with dontLand=1 in fbi
		SPACECRAFT_HOVERING
	};
	enum CollisionState {
		COLLISION_NOUNIT = 0,
		COLLISION_DIRECT = 1, // "directly on path"
		COLLISION_NEARBY = 2, // "generally nearby"
	};

	ASpaceMoveType(CUnit* unit);
	virtual ~ASpaceMoveType() {}

	virtual bool Update();
	// virtual void UpdateLanded();
	virtual void Takeoff() {}
	// virtual void Land() {}
	virtual void SetState(SpacecraftMovementState state) {}
	virtual SpacecraftMovementState GetLandingState() const { return SPACECRAFT_LANDING; }

	float currentPitch = 0.0f;
	bool bankingAllowed = true;
	float currentBank = 0.0f;

	bool CanApplyImpulse(const float3&) { return true; }

	void DependentDied(CObject* o);

protected:
	void CheckForCollision();

public:
	SpacecraftMovementState spacecraftState = SPACECRAFT_FLYING;
	CollisionState collisionState = COLLISION_NOUNIT;

	/// goalpos to resume flying to after landing
	float3 oldGoalPos;

	float accRate = 1.0f;
	float decRate = 1.0f;

	/// mods can use this to disable plane collisions
	bool collide = true;

protected:
	/// unit found to be dangerously close to our path
	CUnit* lastCollidee = nullptr;

	unsigned int crashExpGenID = -1u;
};

#endif // A_AIR_MOVE_TYPE_H_
