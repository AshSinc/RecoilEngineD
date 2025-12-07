/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */


#include "HoverSpaceMoveType.h"
#include "Game/Players/Player.h"
#include "Map/Ground.h"
#include "Map/MapInfo.h"
#include "Sim/Misc/GeometricObjects.h"
#include "Sim/Misc/GlobalSynced.h"
#include "Sim/Misc/GroundBlockingObjectMap.h"
#include "Sim/Misc/QuadField.h"
#include "Sim/Misc/ModInfo.h"
#include "Sim/Units/Scripts/UnitScript.h"
#include "Sim/Units/Unit.h"
#include "Sim/Units/UnitDef.h"
#include "Sim/Units/CommandAI/CommandAI.h"
#include "System/SpringMath.h"
#include "System/Matrix44f.h"
#include "System/SpringHash.h"

#include "System/Misc/TracyDefs.h"
#include <algorithm>
#include <iostream>

CR_BIND_DERIVED(HoverSpaceMoveType, ASpaceMoveType, (nullptr))

CR_REG_METADATA(HoverSpaceMoveType, (
	CR_MEMBER(flyState),
	CR_MEMBER(bankingAllowed),
	CR_MEMBER(airStrafe),
	CR_MEMBER(wantToStop),

	CR_MEMBER(goalDistance),

	CR_MEMBER(currentBank),
	CR_MEMBER(currentPitch),

	CR_MEMBER(turnRate),
	CR_MEMBER(maxDrift),
	CR_MEMBER(maxTurnAngle),

	CR_MEMBER(wantedSpeed),
	CR_MEMBER(deltaSpeed),

	CR_MEMBER(circlingPos),
	CR_MEMBER(randomWind),

	CR_MEMBER(forceHeading),

	CR_MEMBER(wantedHeading),
	CR_MEMBER(forcedHeading),

	CR_MEMBER(waitCounter),
	CR_MEMBER(lastMoveRate),

	CR_PREALLOC(GetPreallocContainer)
))



#define MEMBER_CHARPTR_HASH(memberName) spring::LiteHash(memberName, strlen(memberName),     0)
#define MEMBER_LITERAL_HASH(memberName) spring::LiteHash(memberName, sizeof(memberName) - 1, 0)

static const unsigned int BOOL_MEMBER_HASHES[] = {
	MEMBER_LITERAL_HASH(       "collide"),
	//MEMBER_LITERAL_HASH(      "dontLand"),
	MEMBER_LITERAL_HASH(     "airStrafe"),
	MEMBER_LITERAL_HASH( "useSmoothMesh"),
	MEMBER_LITERAL_HASH("bankingAllowed"),
};
static const unsigned int FLOAT_MEMBER_HASHES[] = {
	MEMBER_LITERAL_HASH( "wantedHeight"),
	MEMBER_LITERAL_HASH(      "accRate"),
	MEMBER_LITERAL_HASH(      "decRate"),
	MEMBER_LITERAL_HASH(     "turnRate"),
	MEMBER_LITERAL_HASH( "altitudeRate"),
	MEMBER_LITERAL_HASH(  "currentBank"),
	MEMBER_LITERAL_HASH( "currentPitch"),
	MEMBER_LITERAL_HASH(     "maxDrift"),
};

#undef MEMBER_CHARPTR_HASH
#undef MEMBER_LITERAL_HASH



static bool UnitIsBusy(const CCommandAI* cai) {
	// queued move-commands (or active build/repair/etc-commands) mean unit has to stay airborne
	return (cai->inCommand != CMD_STOP || cai->HasMoreMoveCommands(false));
}

static bool UnitHasLoadCmd(const CCommandAI* cai) {
	const auto& que = cai->commandQue;
	const auto& cmd = (que.empty())? Command(CMD_STOP): que.front();

	// NOTE:
	//   CMD_LOAD_ONTO is not tested here, HAMT is rarely a transport*ee*
	//   and only transport*er*s can be given the CMD_LOAD_UNITS command
	return (cmd.GetID() == CMD_LOAD_UNITS);
}

static bool UnitIsBusy(const CUnit* u) { return (UnitIsBusy(u->commandAI)); }
static bool UnitHasLoadCmd(const CUnit* u) { return (UnitHasLoadCmd(u->commandAI)); }


//extern ASpaceMoveType::GetGroundHeightFunc amtGetGroundHeightFuncs[6];
extern ASpaceMoveType::EmitCrashTrailFunc amtEmitCrashTrailFuncs_Space[2];



HoverSpaceMoveType::HoverSpaceMoveType(CUnit* owner) :
	ASpaceMoveType(owner),
	flyState(FLY_CRUISING),

	
	// bankingAllowed(true),
	airStrafe(owner != nullptr ? owner->unitDef->airStrafe : false),
	wantToStop(false),

	//goalDistance(1.0f),

	// we want to take off in direction of factory facing
	//currentBank(0.0f),
	//currentPitch(0.0f),

	turnRate(1.0f),
	maxDrift(1.0f),
	maxTurnAngle(math::cos((owner != nullptr ? owner->unitDef->turnInPlaceAngleLimit : 0.0f) * math::DEG_TO_RAD) * -1.0f),

	wantedSpeed(ZeroVector),
	deltaSpeed(ZeroVector),
	circlingPos(ZeroVector),
	randomWind(ZeroVector),

	forceHeading(false),

	wantedHeading(owner != nullptr ? GetHeadingFromFacing(owner->buildFacing) : 0),
	forcedHeading(wantedHeading),

	waitCounter(0),
	lastMoveRate(0)
{
	// creg
	if (owner == nullptr)
		return;

	assert(owner->unitDef != nullptr);

	turnRate = owner->unitDef->turnRate;

	//wantedHeight = owner->unitDef->wantedHeight + gsRNG.NextFloat() * 5.0f;
	//orgWantedHeight = wantedHeight;

	bankingAllowed = owner->unitDef->bankingAllowed;
	currentBank = 0.0f;

	// prevent weapons from being updated and firing while on the ground
	owner->SetHoldFire(true);
}


void HoverSpaceMoveType::SetGoal(const float3& pos, float distance)
{
	RECOIL_DETAILED_TRACY_ZONE;
	goalPos = pos;
	oldGoalPos = pos;

	// aircraft need some marginals to avoid uber stacking
	// when lots of them are ordered to one place
	maxDrift = std::max(16.0f, distance);
	//wantedHeight = orgWantedHeight;

	forceHeading = false;
}

void HoverSpaceMoveType::SetState(SpacecraftMovementState newState)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// once in crashing, we should never change back into another state
	if (spacecraftState == SPACECRAFT_CRASHING && newState != SPACECRAFT_CRASHING)
		return;

	if (newState == spacecraftState)
		return;


	owner->onTempHoldFire = (newState == SPACECRAFT_LANDED);
	owner->useAirLos = (newState != SPACECRAFT_LANDED);

	spacecraftState = newState;

	switch (spacecraftState) {
		case SPACECRAFT_CRASHING:
			owner->SetPhysicalStateBit(CSolidObject::PSTATE_BIT_CRASHING);
			break;

		#if 0
		case SPACECRAFT_FLYING:
			owner->Activate();
			break;
		case SPACECRAFT_LANDING:
			owner->Deactivate();
			break;
		#endif

		case SPACECRAFT_LANDED:
			// FIXME already inform commandAI in AIRCRAFT_LANDING!
			owner->commandAI->StopMove();

			owner->Deactivate();
			owner->Block();
			owner->ClearPhysicalStateBit(CSolidObject::PSTATE_BIT_FLYING);
			break;
		case SPACECRAFT_TAKEOFF:
			owner->Activate();
			owner->UnBlock();
			owner->SetPhysicalStateBit(CSolidObject::PSTATE_BIT_FLYING);
			break;
		case SPACECRAFT_HOVERING: {
			// when heading is forced by TCAI we are busy (un-)loading
			// a unit and do not want wantedHeight to be tampered with
			//wantedHeight = mix(orgWantedHeight, wantedHeight, forceHeading);
			wantedSpeed = ZeroVector;
		} // fall through
		default:
			//ClearLandingPos();
			break;
	}

	// Cruise as default
	// FIXME: RHS overrides FLY_ATTACKING and FLY_CIRCLING (via UpdateFlying-->ExecuteStop)
	if (spacecraftState == SPACECRAFT_FLYING || spacecraftState == SPACECRAFT_HOVERING)
		flyState = FLY_CRUISING;

	owner->UpdatePhysicalStateBit(CSolidObject::PSTATE_BIT_MOVING, (spacecraftState != SPACECRAFT_LANDED));
	waitCounter = 0;
}

// void CHoverAirMoveType::SetAllowLanding(bool allowLanding)
// {
// 	RECOIL_DETAILED_TRACY_ZONE;
// 	dontLand = !allowLanding;

// 	if (CanLand(false))
// 		return;

// 	if (aircraftState != AIRCRAFT_LANDED && aircraftState != AIRCRAFT_LANDING)
// 		return;

// 	// do not start hovering if still (un)loading a unit
// 	if (forceHeading)
// 		return;

// 	SetState(AIRCRAFT_HOVERING);
// }

void HoverSpaceMoveType::StartMoving(float3 pos, float goalRadius)
{
	RECOIL_DETAILED_TRACY_ZONE;
	forceHeading = false;
	wantToStop = false;
	waitCounter = 0;

	owner->SetPhysicalStateBit(CSolidObject::PSTATE_BIT_MOVING);

	switch (spacecraftState) {
		case SPACECRAFT_LANDED:
			SetState(SPACECRAFT_TAKEOFF);
			break;
		case SPACECRAFT_TAKEOFF:
			SetState(SPACECRAFT_TAKEOFF);
			break;
		case SPACECRAFT_FLYING:
			flyState = FLY_CRUISING;
			break;
		case SPACECRAFT_LANDING:
			SetState(SPACECRAFT_TAKEOFF);
			break;
		case SPACECRAFT_HOVERING:
			SetState(SPACECRAFT_FLYING);
			break;
		case SPACECRAFT_CRASHING:
			break;
	}

	SetGoal(pos, goalRadius);
	progressState = AMoveType::Active;
}

void HoverSpaceMoveType::StartMoving(float3 pos, float goalRadius, float speed)
{
	RECOIL_DETAILED_TRACY_ZONE;
	StartMoving(pos, goalRadius);
}

//Can't remove this yet, overrides from abstract further up
void HoverSpaceMoveType::KeepPointingTo(float3 pos, float distance, bool aggressive)
{
	RECOIL_DETAILED_TRACY_ZONE;
	wantToStop = false;
	forceHeading = false;
	//wantedHeight = orgWantedHeight;

	// close in a little to avoid the command AI overriding pos constantly
	distance -= 15.0f;

	// Ignore the exact same order
	if ((spacecraftState == SPACECRAFT_FLYING) && (flyState == FLY_CIRCLING || flyState == FLY_ATTACKING) && ((circlingPos - pos).SqLength2D() < 64) && (goalDistance == distance))
		return;

	circlingPos = pos;
	goalPos = owner->pos;

	// let this handle any needed state transitions
	StartMoving(goalPos, goalDistance = distance);

	// FIXME:
	//   the FLY_ATTACKING state is broken (unknown how long this has been
	//   the case because <aggressive> was always false up until e7ae68df)
	//   aircraft fly *right up* to their target without stopping
	//   after fixing this, the "circling" behavior is now broken
	//   (waitCounter is always 0)
	if (aggressive) {
		flyState = FLY_ATTACKING;
	} else {
		flyState = FLY_CIRCLING;
	}
}

void HoverSpaceMoveType::ExecuteStop()
{
	RECOIL_DETAILED_TRACY_ZONE;
	wantToStop = false;
	wantedSpeed = ZeroVector;

	SetGoal(owner->pos);
	//ClearLandingPos();

	switch (spacecraftState) {
		// case AIRCRAFT_TAKEOFF: {
		// 	if (CanLand(UnitIsBusy(owner))) {
		// 		SetState(AIRCRAFT_LANDING);
		// 		// trick to land directly
		// 		waitCounter = GAME_SPEED;
		// 		break;
		// 	}
		// } // fall through
		case SPACECRAFT_FLYING: {
			// if (CanLand(UnitIsBusy(owner))) {
			// 	SetState(SPACECRAFT_LANDING);
			// } else {
				SetState(SPACECRAFT_HOVERING);
			//}
		} break;

		// case AIRCRAFT_LANDING: {
		// 	if (!CanLand(UnitIsBusy(owner)))
		// 		SetState(AIRCRAFT_HOVERING);

		// } break;
		// case AIRCRAFT_LANDED: {} break;
		case SPACECRAFT_CRASHING: {} break;

		case SPACECRAFT_HOVERING: {
			// if (CanLand(UnitIsBusy(owner))) {
			// 	// land immediately, otherwise keep hovering
			// 	SetState(SPACECRAFT_LANDING);
			// 	waitCounter = GAME_SPEED;
			// }
			SetGoal(owner->pos);
			wantedSpeed = ZeroVector;
		} break;
	}
}

void HoverSpaceMoveType::StopMoving(bool callScript, bool hardStop, bool)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// transports switch to landed state (via SetState which calls
	// us) during pickup but must *not* be allowed to change their
	// heading while "landed" (see MobileCAI)
	forceHeading &= (spacecraftState == SPACECRAFT_LANDED);
	wantToStop = true;
	//wantedHeight = orgWantedHeight;

	owner->ClearPhysicalStateBit(CSolidObject::PSTATE_BIT_MOVING);

	if (progressState != AMoveType::Failed)
		progressState = AMoveType::Done;
}



// void CSpaceMoveType::UpdateLanded()
// {
// 	RECOIL_DETAILED_TRACY_ZONE;
// 	AAirMoveType::UpdateLanded();

// 	if (progressState != AMoveType::Failed)
// 		progressState = AMoveType::Done;
// }

// void CSpaceMoveType::UpdateTakeoff()
// {
// 	RECOIL_DETAILED_TRACY_ZONE;
// 	const float3& pos = owner->pos;

// 	wantedSpeed = ZeroVector;
// 	wantedHeight = orgWantedHeight;

// 	UpdateAirPhysics();

// 	const float curAltitude = pos.y - amtGetGroundHeightFuncs[canSubmerge](pos.x, pos.z);
// 	const float minAltitude = orgWantedHeight * 0.8f;

// 	if (curAltitude <= minAltitude)
// 		return;

// 	SetState(AIRCRAFT_FLYING);
// }


// Move the unit around a bit..
void HoverSpaceMoveType::UpdateHovering()
{
	const float  curSqGoalDist = goalPos.SqDistance2D(owner->pos);
	const float  maxSqGoalDist = Square(GetGoalRadius());
	const float absHoverFactor = math::fabs(owner->unitDef->dlHoverFactor) * 0.5f;

	// randomWind.x = (randomWind.x * 0.9f + (gsRNG.NextFloat() - 0.5f) * 0.5f);
	// randomWind.y = (randomWind.y * 0.9f + (gsRNG.NextFloat() - 0.5f) * 0.5f);
	// randomWind.z = (randomWind.z * 0.9f + (gsRNG.NextFloat() - 0.5f) * 0.5f);

	//randomWind = ZeroVector;

	// randomly drift (but not too far from goal-position; a larger
	// deviation causes a larger wantedSpeed back in its direction)
	// when not picking up a transportee and when not close to goal
	// otherwise any aircraft that entered state=HOVERING while its
	// move order is still unfinished might not (ever) transition to
	// LANDING
	// wantedSpeed = (randomWind * absHoverFactor * (1 - UnitHasLoadCmd(owner)) * (curSqGoalDist > maxSqGoalDist));
	// wantedSpeed += (smoothstep(0.0f, 20.0f * 20.0f, float3::fabs(goalPos - owner->pos)) * (goalPos - owner->pos));
	// wantedSpeed = float3::min(float3::fabs(wantedSpeed), OnesVector * maxSpeed) * float3::sign(wantedSpeed);

	//ISSUE above causes a strange pushback


	UpdateAirPhysics();
}


void HoverSpaceMoveType::UpdateFlying()
{
	// std::cout << "In Update Flying in space!" << "\n";
	RECOIL_DETAILED_TRACY_ZONE;
	const float3& pos = owner->pos;
	// const float4& spd = owner->speed;

	// Direction to where we would like to be
	float3 goalVec = goalPos - pos;
	float3 goalDir = goalVec;

	// don't change direction for waypoints we just flew over and missed slightly
	if (flyState != FLY_LANDING && owner->commandAI->HasMoreMoveCommands()) {
		if ((goalDir != ZeroVector) && (goalVec.dot(goalDir.UnsafeANormalize()) < 1.0f))
			goalVec = owner->frontdir;
	}

	//const float goalDistSq2D = goalVec.SqLength2D();
	const float goalDistSq = goalVec.SqLength();
	//const float groundHeight = amtGetGroundHeightFuncs[4 * UseSmoothMesh()](pos.x, pos.z);

	const bool closeToGoal = (flyState == FLY_ATTACKING)?
		(goalDistSq < (             400.0f)):
		// (goalDistSq2D < (maxDrift * maxDrift)) && (math::fabs(groundHeight + wantedHeight - pos.y) < maxDrift);
		(goalDistSq < (maxDrift * maxDrift));// && (math::fabs(wantedHeight - pos.y) < maxDrift);

	if (closeToGoal) {
		switch (flyState) {
			case FLY_CRUISING: {
				const bool isTransporter = owner->unitDef->IsTransportUnit();
				const bool hasLoadCmds = isTransporter && UnitHasLoadCmd(owner);

				// [?] transport aircraft need some time to detect that they can pickup
				const bool canLoad = isTransporter && (++waitCounter < ((GAME_SPEED << 1) - 5));
				const bool isBusy = UnitIsBusy(owner);

				//if (!CanLand(isBusy) || (canLoad && hasLoadCmds)) {
				// if ((canLoad && hasLoadCmds)) {
				// 	wantedSpeed = ZeroVector;

				// 	// if (isTransporter) {
				// 	// 	if (waitCounter > (GAME_SPEED << 1))
				// 	// 		wantedHeight = orgWantedHeight;

				// 	// 	SetState(SPACECRAFT_HOVERING);
				// 	// } else {
				// 		if (!isBusy) {
				// 			wantToStop = true;

				// 			// NOTE:
				// 			//   this is not useful, next frame UpdateFlying()
				// 			//   will change it to _LANDING because wantToStop
				// 			//   is now true
				// 			SetState(SPACECRAFT_HOVERING);
				// 		}
				// 	//}
				// } else {
				// 	//wantedHeight = orgWantedHeight;
				// 	SetState(SPACECRAFT_LANDING);
				// }
			} break;

			case FLY_CIRCLING: {
				if ((++waitCounter) > ((GAME_SPEED * 3) + 10)) {
					if (airStrafe) {
						float3 relPos = pos - circlingPos;

						if (relPos.x < 0.0001f && relPos.x > -0.0001f)
							relPos.x = 0.0001f;

						static const CMatrix44f rotCCW(0.0f, math::QUARTERPI, 0.0f);
						static const CMatrix44f rotCW(0.0f, -math::QUARTERPI, 0.0f);

						// make sure the point is on the circle, go there in a straight line
						if (gsRNG.NextFloat() > 0.5f) {
							goalPos = circlingPos + (rotCCW.Mul(relPos.Normalize2D()) * goalDistance);
						} else {
							goalPos = circlingPos + (rotCW.Mul(relPos.Normalize2D()) * goalDistance);
						}
					}
					waitCounter = 0;
				}
			} break;

			case FLY_ATTACKING: {
				if (airStrafe) {
					float3 relPos = pos - circlingPos;

					if (relPos.x < 0.0001f && relPos.x > -0.0001f)
						relPos.x = 0.0001f;

					const float rotY = 0.6f + gsRNG.NextFloat() * 0.6f;
					const float sign = (gsRNG.NextFloat() > 0.5f) ? 1.0f : -1.0f;
					const CMatrix44f rot(0.0f, rotY * sign, 0.0f);

					// Go there in a straight line
					goalPos = circlingPos + (rot.Mul(relPos.Normalize()) * goalDistance);
				}
			} break;

			// case FLY_LANDING: {
			// } break;
		}
	}

	// not "close" to goal yet, so keep going
	// use 2D math since goal is on the ground
	// but we are not
	//goalVec.y = 0.0f;

	// if we are close to our goal, we should
	// adjust speed st. we never overshoot it
	// (by respecting current brake-distance)
	//const float curSpeed = owner->speed.Length2D();
	const float curSpeed = owner->speed.Length();
	const float brakeDist = (0.5f * curSpeed * curSpeed) / decRate;
	const float goalDist = goalVec.Length() + 0.1f;
	const float goalSpeed =
		(maxSpeed          ) * (goalDist >  brakeDist) +
		(curSpeed - decRate) * (goalDist <= brakeDist);

	if (goalDist > goalSpeed) {
		// update our velocity and heading so long as goal is still
		// further away than the distance we can cover in one frame
		// we must use a variable-size "dead zone" to avoid freezing
		// in mid-air or oscillation behavior at very close distances
		// NOTE:
		//   wantedSpeed is a vector, so even aircraft with turnRate=0
		//   are coincidentally able to reach any goal by side-strafing
		wantedHeading = GetHeadingFromVector(goalVec.x, goalVec.z);
		wantedSpeed = (goalVec / goalDist) * std::min(goalSpeed, maxWantedSpeed);
	} else {
		// switch to hovering (if !CanLand()))
		if (!UnitIsBusy(owner)) {
			ExecuteStop();
		} else {
			wantedSpeed = ZeroVector;
		}
	}

	// redundant, done in Update()
	// UpdateHeading();
	UpdateAirPhysics();

	// Point toward goal or forward - unless we just passed it to get to another goal
	if ((flyState == FLY_ATTACKING) || (flyState == FLY_CIRCLING)) {
		goalVec = circlingPos - pos;
	} else {
		const bool b0 = (flyState != FLY_LANDING && (owner->commandAI->HasMoreMoveCommands()));
		const bool b1 = (goalDist < brakeDist && goalDist > 1.0f);

		if (b0 && b1) {
			goalVec = owner->frontdir;
		} else {
			goalVec = goalPos - pos;
		}
	}

	if (goalVec.SqLength() > 1.0f) {
		// update heading again in case goalVec changed
		wantedHeading = GetHeadingFromVector(goalVec.x, goalVec.z);
		//wantedHeading = GetHeadingFromVector(goalVec.x, goalVec.z);
	}
}

void HoverSpaceMoveType::UpdateHeading()
{
	RECOIL_DETAILED_TRACY_ZONE;
	if (spacecraftState == SPACECRAFT_TAKEOFF && !owner->unitDef->factoryHeadingTakeoff)
		return;
	// UpdateDirVectors() resets our up-vector but we
	// might have residual pitch angle from attacking
	// if (aircraftState == AIRCRAFT_LANDING)
	//     return;

	const short refHeading = mix(wantedHeading, forcedHeading, forceHeading);
	const short deltaHeading = refHeading - owner->heading;

	if (deltaHeading > 0) {
		owner->AddHeading(std::min(deltaHeading, short( turnRate)), owner->IsOnGround(), false, 0.0f);
	} else {
		owner->AddHeading(std::max(deltaHeading, short(-turnRate)), owner->IsOnGround(), false, 0.0f);
	}
}

void HoverSpaceMoveType::UpdateBanking(bool noBanking)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// need to allow LANDING so (autoLand=true) aircraft reset their
	// pitch naturally after attacking ground and being told to stop
	if (spacecraftState != SPACECRAFT_FLYING && spacecraftState != SPACECRAFT_HOVERING && spacecraftState != SPACECRAFT_LANDING)
		return;

	// always positive
	const float bankLimit = std::min(1.0f, goalPos.SqDistance2D(owner->pos) * Square(0.15f));

	float wantedBank = 0.0f;
	float wantedPitch = 0.0f;

	SyncedFloat3& frontDir = owner->frontdir;
	SyncedFloat3& upDir = owner->updir;
	SyncedFloat3& rightDir3D = owner->rightdir;
	SyncedFloat3  rightDir2D;

	// pitching does not affect rightdir, but we want a flat right-vector to calculate wantedBank
	frontDir.y = currentPitch;
	frontDir.Normalize();

	rightDir2D = frontDir.cross(UpVector);


	if (!owner->upright) {
		// std::max() is here to guard around the case when circlingPos == owner->pos,
		// which caused NaNs all over the place
		wantedPitch = (goalPos.y - owner->pos.y) / std::max(0.01f, goalPos.distance(owner->pos));
	}

	wantedPitch *= (spacecraftState == SPACECRAFT_FLYING && goalPos.y != owner->pos.y);
	currentPitch = mix(currentPitch, wantedPitch, 0.05f);

	if (!noBanking && bankingAllowed)
		wantedBank = rightDir2D.dot(deltaSpeed) / accRate * 0.5f;
	if (Square(wantedBank) > bankLimit)
		wantedBank = math::sqrt(bankLimit);

	// Adjust our banking to the desired value
	if (currentBank > wantedBank) {
		currentBank -= std::min(0.03f, currentBank - wantedBank);
	} else {
		currentBank += std::min(0.03f, wantedBank - currentBank);
	}


	upDir = rightDir2D.cross(frontDir);
	upDir = upDir * math::cos(currentBank) + rightDir2D * math::sin(currentBank);
	upDir = upDir.Normalize();
	rightDir3D = frontDir.cross(upDir);

	// NOTE:
	//   heading might not be fully in sync with frontDir due to the
	//   vector<-->heading mapping not being 1:1 (such that heading
	//   != GetHeadingFromVector(frontDir)), therefore this call can
	//   cause owner->heading to change --> unwanted if forceHeading
	//
	//   it is "safe" to skip because only frontDir.y is manipulated
	//   above so its xz-direction does not change, but the problem
	//   should really be fixed elsewhere
	if (!forceHeading)
		owner->SetHeadingFromDirection();

	owner->UpdateMidAndAimPos();
}

bool HoverSpaceMoveType::UpdateAirPhysics()
{
	RECOIL_DETAILED_TRACY_ZONE;
	const float3& pos = owner->pos;
	const float4& spd = owner->speed;

	SyncedFloat3& rightdir = owner->rightdir;
	SyncedFloat3& frontdir = owner->frontdir;
	SyncedFloat3& updir    = owner->updir;

	// const float3 straight_path_vector = goalPos - pos;
	// const auto [straight_path_dir, straight_path_length] = straight_path_vector.GetNormalized();

	// const float3 newVelocity = spd + (straight_path_dir * accRate);
	// const auto [newVelocity_dir, newVelocity_length] = newVelocity.GetNormalized();
	// float clamped_length = 0;
	// clamped_length = std::clamp(clamped_length, newVelocity_length, owner->unitDef->speed);

	//wantedSpeed

	//frontdir += (rightdir * yprDeltas.x * yprScales.x); // yaw
	// frontdir += (updir    * yprDeltas.y * yprScales.y); // pitch
	// //updir    += (rightdir * yprDeltas.z * yprScales.z); // roll (via updir, new rightdir derives from it)
	// frontdir += ((speedDir - frontdir) * frontToSpeed);

	//owner->SetVelocity(clamped_length*newVelocity_dir);
	//owner->SetVelocity(wantedSpeed);

	bool nextPosInBounds = true;

	bool crashed = false;

	{
		const float3 deltaSpeed = wantedSpeed - spd;

		// apply {acc,dec}eleration as wanted
		const float deltaSpeedSqr = deltaSpeed.SqLength();
		const float deltaSpeedRate = mix(accRate, decRate, deltaSpeed.dot(spd) < 0.0f);

		if (deltaSpeedSqr < Square(deltaSpeedRate)) {
			//std::cout << "Here1" << "\n";
			owner->SetVelocity(wantedSpeed);
		} else {
			owner->SetVelocity(spd + (deltaSpeed / math::sqrt(deltaSpeedSqr) * deltaSpeedRate));
		}
	}

	if (nextPosInBounds)
		owner->Move(spd, true);

	frontdir.Normalize();
	rightdir = frontdir.cross(updir);
	rightdir.Normalize();
	updir = rightdir.cross(frontdir);
	updir.Normalize();

	owner->SetSpeed(spd);
	owner->UpdateMidAndAimPos();
	owner->SetHeadingFromDirection();
	return crashed;
}

void HoverSpaceMoveType::UpdateMoveRate()
{
	RECOIL_DETAILED_TRACY_ZONE;
	int curMoveRate = 1;

	// currentspeed is not used correctly for vertical movement, so compensate with this hax
	if (spacecraftState != SPACECRAFT_LANDING && spacecraftState != SPACECRAFT_TAKEOFF)
		curMoveRate = CalcScriptMoveRate(owner->speed.w, 3.0f);

	if (curMoveRate == lastMoveRate)
		return;

	owner->script->MoveRate(lastMoveRate = curMoveRate);
}


bool HoverSpaceMoveType::Update()
{
	RECOIL_DETAILED_TRACY_ZONE;
	const float3 lastPos = owner->pos;
	const float4 lastSpd = owner->speed;

	ASpaceMoveType::Update();

	if ((owner->IsStunned() && !owner->IsCrashing()) || owner->beingBuilt) {
		wantedSpeed = ZeroVector;

		UpdateAirPhysics();
		return (HandleCollisions(collide && !owner->beingBuilt && (spacecraftState != SPACECRAFT_TAKEOFF)));
	}

	// allow us to stop if wanted (changes aircraft state)
	if (wantToStop)
		ExecuteStop();

	switch (spacecraftState) {
		case SPACECRAFT_LANDED:
			//UpdateLanded();
			break;
		case SPACECRAFT_TAKEOFF:
			//UpdateTakeoff();
			break;
		case SPACECRAFT_FLYING:
			UpdateFlying();
			//UpdateHovering();
			break;
		case SPACECRAFT_LANDING:
			//UpdateLanding();
			break;
		case SPACECRAFT_HOVERING:
			UpdateAirPhysics();
			//UpdateHovering(); //wind code causes huge knockback for some reason
			break;
		case SPACECRAFT_CRASHING: {
			if (UpdateAirPhysics()
					|| (CGround::GetHeightAboveWater(owner->pos.x, owner->pos.z) + 5.0f + owner->radius) > owner->pos.y){
				owner->ForcedKillUnit(nullptr, true, false, -CSolidObject::DAMAGE_AIRCRAFT_CRASHED);
			} else {
				#define SPIN_DIR(o) ((o->id & 1) * 2 - 1)
				wantedHeading = GetHeadingFromVector(owner->rightdir.x * SPIN_DIR(owner), owner->rightdir.z * SPIN_DIR(owner));
				//wantedHeight = 0.0f;
				#undef SPIN_DIR
			}

			amtEmitCrashTrailFuncs_Space[crashExpGenID != -1u](owner, crashExpGenID);
		} break;
	}

	if (lastSpd == ZeroVector && owner->speed != ZeroVector) { owner->script->StartMoving(false); }
	if (lastSpd != ZeroVector && owner->speed == ZeroVector) { owner->script->StopMoving(); }

	// Banking requires deltaSpeed.y = 0
	deltaSpeed = owner->speed - lastSpd;
	deltaSpeed.y = 0.0f;

	// Turn and bank and move; update dirs
	UpdateHeading();
	UpdateBanking(spacecraftState == SPACECRAFT_HOVERING || spacecraftState == SPACECRAFT_LANDING);

	return (HandleCollisions(collide && !owner->beingBuilt && (spacecraftState != SPACECRAFT_TAKEOFF)));
}

void HoverSpaceMoveType::SlowUpdate()
{
	RECOIL_DETAILED_TRACY_ZONE;
	UpdateMoveRate();
	// note: NOT AAirMoveType::SlowUpdate
	AMoveType::SlowUpdate();
}

void HoverSpaceMoveType::ForceHeading(short h)
{
	RECOIL_DETAILED_TRACY_ZONE;
	forceHeading = true;
	forcedHeading = h;
}

void HoverSpaceMoveType::Takeoff()
{
	RECOIL_DETAILED_TRACY_ZONE;
	if (spacecraftState == ASpaceMoveType::SPACECRAFT_LANDED) {
		SetState(ASpaceMoveType::SPACECRAFT_TAKEOFF);
	}
	if (spacecraftState == ASpaceMoveType::SPACECRAFT_LANDING) {
		SetState(ASpaceMoveType::SPACECRAFT_FLYING);
	}
}

bool HoverSpaceMoveType::HandleCollisions(bool checkCollisions)
{
	RECOIL_DETAILED_TRACY_ZONE;
	const float3& pos = owner->pos;

	if (pos != oldPos) {
		oldPos = pos;

		bool hitBuilding = false;

		// check for collisions if not being built or not taking off
		// includes an extra condition for transports, which are exempt while loading
		if (!forceHeading && checkCollisions) {
			QuadFieldQuery qfQuery;
			quadField.GetUnitsExact(qfQuery, pos, owner->radius + 6);

			for (CUnit* unit: *qfQuery.units) {
				const bool unloadingUnit  = ( unit->unloadingTransportId == owner->id);
				const bool unloadingOwner = (owner->unloadingTransportId ==  unit->id);
				const bool   loadingUnit  = ( unit->id == owner->loadingTransportId);
				const bool   loadingOwner = (owner->id ==  unit->loadingTransportId);

				if (unloadingUnit)
					unit->unloadingTransportId = -1;
				if (unloadingOwner)
					owner->unloadingTransportId = -1;

				if (loadingUnit || loadingOwner || unit == owner->transporter || unit->transporter != nullptr)
					continue;


				const float sqDist = (pos - unit->pos).SqLength();
				const float totRad = owner->radius + unit->radius;

				if (sqDist <= 0.1f || sqDist >= (totRad * totRad))
					continue;

				//Keep them marked as recently unloaded
				if (unloadingUnit) {
					unit->unloadingTransportId = owner->id;
					continue;
				}

				if (unloadingOwner) {
					owner->unloadingTransportId = unit->id;
					continue;
				}


				const float dist = math::sqrt(sqDist);
				const float3 dif = (pos - unit->pos).Normalize();

				if (unit->mass >= CSolidObject::DEFAULT_MASS || unit->immobile) {
					owner->Move(-dif * (dist - totRad), true);
					owner->SetVelocity(owner->speed * 0.99f);

					hitBuilding = true;
				} else {
					const float part = owner->mass / (owner->mass + unit->mass);
					const float colSpeed = -owner->speed.dot(dif) + unit->speed.dot(dif);

					owner->Move(-dif * (dist - totRad) * (1.0f - part), true);
					owner->SetVelocity(owner->speed + (dif * colSpeed * (1.0f - part)));

					if (!unit->UsingScriptMoveType()) {
						unit->SetVelocityAndSpeed(unit->speed - (dif * colSpeed * (part)));
						unit->Move(dif * (dist - totRad) * (part), true);
					}
				}
			}

			// update speed.w
			owner->SetSpeed(owner->speed);
		}

		if (hitBuilding && owner->IsCrashing()) {
			owner->ForcedKillUnit(nullptr, true, false, -CSolidObject::DAMAGE_AIRCRAFT_CRASHED);
			return true;
		}

		if (pos.x < 0.0f) {
			owner->Move( RgtVector * 0.6f, true);
		} else if (pos.x > float3::maxxpos) {
			owner->Move(-RgtVector * 0.6f, true);
		}

		if (pos.z < 0.0f) {
			owner->Move( FwdVector * 0.6f, true);
		} else if (pos.z > float3::maxzpos) {
			owner->Move(-FwdVector * 0.6f, true);
		}

		return true;
	}

	return false;
}



bool HoverSpaceMoveType::SetMemberValue(unsigned int memberHash, void* memberValue) {
	RECOIL_DETAILED_TRACY_ZONE;
	// try the generic members first
	if (AMoveType::SetMemberValue(memberHash, memberValue))
		return true;

	#define     DONTLAND_MEMBER_IDX 1
	#define WANTEDHEIGHT_MEMBER_IDX 0

	// unordered_map etc. perform dynallocs, so KISS here
	bool* boolMemberPtrs[] = {
		&collide,
		//&dontLand,
		&airStrafe,

		//&useSmoothMesh,
		&bankingAllowed
	};
	float* floatMemberPtrs[] = {
		//&wantedHeight,

		&accRate,
		&decRate,

		&turnRate,
		//&altitudeRate,

		&currentBank,
		&currentPitch,

		&maxDrift
	};

	// special cases
	if (memberHash == BOOL_MEMBER_HASHES[DONTLAND_MEMBER_IDX]) {
		//SetAllowLanding(!(*(reinterpret_cast<bool*>(memberValue))));
		return true;
	}
	if (memberHash == FLOAT_MEMBER_HASHES[WANTEDHEIGHT_MEMBER_IDX]) {
		//SetDefaultAltitude(*(reinterpret_cast<float*>(memberValue)));
		return true;
	}

	// note: <memberHash> should be calculated via HsiehHash
	for (unsigned int n = 0; n < sizeof(boolMemberPtrs) / sizeof(boolMemberPtrs[0]); n++) {
		if (memberHash == BOOL_MEMBER_HASHES[n]) {
			*(boolMemberPtrs[n]) = *(reinterpret_cast<bool*>(memberValue));
			return true;
		}
	}

	for (unsigned int n = 0; n < sizeof(floatMemberPtrs) / sizeof(floatMemberPtrs[0]); n++) {
		if (memberHash == FLOAT_MEMBER_HASHES[n]) {
			*(floatMemberPtrs[n]) = *(reinterpret_cast<float*>(memberValue));
			return true;
		}
	}

	return false;
}

