/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

#include "ASpaceMoveType.h"

#include "Components/MoveTypesComponents.h"
#include "Game/GlobalUnsynced.h"
#include "Map/Ground.h"
#include "Map/MapInfo.h"
#include "Rendering/Env/Particles/Classes/SmokeProjectile.h"
#include "Sim/Ecs/Registry.h"
#include "Sim/Misc/QuadField.h"
#include "Sim/Misc/SmoothHeightMesh.h"
#include "Sim/Projectiles/ExplosionGenerator.h"
#include "Sim/Projectiles/ProjectileMemPool.h"
#include "Sim/Units/Unit.h"
#include "Sim/Units/UnitDef.h"
#include "Sim/Units/CommandAI/CommandAI.h"
#include "System/SpringMath.h"

#include "System/Misc/TracyDefs.h"

using namespace MoveTypes;

CR_BIND_DERIVED_INTERFACE(ASpaceMoveType, AMoveType)

CR_REG_METADATA(ASpaceMoveType, (
	CR_MEMBER(spacecraftState),
	CR_MEMBER(collisionState),
	CR_MEMBER(oldGoalPos),
	CR_MEMBER(accRate),
	CR_MEMBER(decRate),
	CR_MEMBER(collide),
	CR_MEMBER(lastCollidee),
	CR_MEMBER(crashExpGenID)
))

static inline float AAMTGetGroundHeightAW(float x, float z) { return CGround::GetHeightAboveWater(x, z); }
static inline float AAMTGetGroundHeight  (float x, float z) { return CGround::GetHeightReal      (x, z); }
static inline float AAMTGetSmoothGroundHeightAW(float x, float z) { return smoothGround.GetHeightAboveWater(x, z); }
static inline float AAMTGetSmoothGroundHeight  (float x, float z) { return smoothGround.GetHeight          (x, z); }
static inline float HAMTGetMaxGroundHeight(float x, float z) { return std::max(smoothGround.GetHeight(x, z), CGround::GetApproximateHeight(x, z)); }
static inline float SAMTGetMaxGroundHeight(float x, float z) { return std::max(smoothGround.GetHeight(x, z), CGround::GetHeightAboveWater(x, z)); }

static inline void AAMTEmitEngineTrail(CUnit* owner, unsigned int) {
	projMemPool.alloc<CSmokeProjectile>(owner, owner->midPos, guRNG.NextVector() * 0.08f, (100.0f + guRNG.NextFloat() * 50.0f), 5.0f, 0.2f, 0.4f);
}
static inline void AAMTEmitCustomTrail(CUnit* owner, unsigned int id) {
	explGenHandler.GenExplosion(
		id,
		owner->midPos,
		owner->frontdir,
		1.0f,
		0.0f,
		1.0f,
		owner,
		ExplosionHitObject()
	);
}

ASpaceMoveType::EmitCrashTrailFunc amtEmitCrashTrailFuncs_Space[2] = {
	AAMTEmitEngineTrail,
	AAMTEmitCustomTrail,
};

ASpaceMoveType::ASpaceMoveType(CUnit* unit): AMoveType(unit)
{
	RECOIL_DETAILED_TRACY_ZONE;
	// creg
	if (unit == nullptr)
		return;

	const UnitDef* ud = owner->unitDef;

	oldGoalPos = unit->pos;

	accRate = std::max(0.01f, ud->maxAcc);
	decRate = std::max(0.01f, ud->maxDec);

	collide = ud->collide;

	UseHeading(false);

	if (ud->GetCrashExpGenCount() > 0) {
		crashExpGenID = guRNG.NextInt(ud->GetCrashExpGenCount());
		crashExpGenID = ud->GetCrashExpGenID(crashExpGenID);
	}

	Sim::registry.emplace_or_replace<GeneralMoveType>(owner->entityReference, owner->id);
}

void ASpaceMoveType::DependentDied(CObject* o) {
	RECOIL_DETAILED_TRACY_ZONE;
	if (o == lastCollidee) {
		lastCollidee = nullptr;
		collisionState = COLLISION_NOUNIT;
	}
}

bool ASpaceMoveType::Update() {
	RECOIL_DETAILED_TRACY_ZONE;
	// NOTE: useHeading is never true by default for aircraft (AAirMoveType
	// forces it to false, while only CUnit::{Attach,Detach}Unit manipulate
	// it specifically for HoverAirMoveType's)
	if (UseHeading()) {
		SetState(SPACECRAFT_TAKEOFF);
		UseHeading(false);
	}

	// prevent UnitMoved event spam
	return false;
}


void ASpaceMoveType::CheckForCollision()
{
	RECOIL_DETAILED_TRACY_ZONE;
	if (!collide)
		return;

	const SyncedFloat3& pos = owner->midPos;
	const SyncedFloat3& forward = owner->frontdir;

	float dist = 200.0f;

	QuadFieldQuery qfQuery;
	quadField.GetUnitsExact(qfQuery, pos + forward * 121.0f, dist);

	if (lastCollidee != nullptr) {
		DeleteDeathDependence(lastCollidee, DEPENDENCE_LASTCOLWARN);

		lastCollidee = nullptr;
		collisionState = COLLISION_NOUNIT;
	}

	// find closest potential collidee
	for (CUnit* unit: *qfQuery.units) {
		if (unit == owner || !unit->unitDef->canfly)
			continue;

		const SyncedFloat3& op = unit->midPos;
		const float3 dif = op - pos;
		const float3 forwardDif = forward * (forward.dot(dif));

		if (forwardDif.SqLength() >= (dist * dist))
			continue;

		const float3 ortoDif = dif - forwardDif;
		const float frontLength = forwardDif.Length();
		// note: radii are multiplied by two
		const float minOrtoDif = (unit->radius + owner->radius) * 2.0f + frontLength * 0.1f + 10.0f;

		if (ortoDif.SqLength() < (minOrtoDif * minOrtoDif)) {
			dist = frontLength;
			lastCollidee = const_cast<CUnit*>(unit);
		}
	}

	if (lastCollidee != nullptr) {
		collisionState = COLLISION_DIRECT;
		AddDeathDependence(lastCollidee, DEPENDENCE_LASTCOLWARN);
		return;
	}

	for (CUnit* u: *qfQuery.units) {
		if (u == owner)
			continue;

		if ((u->midPos - pos).SqLength() > Square((owner->radius + u->radius) * 2.0f))
			continue;

		lastCollidee = u;
	}

	if (lastCollidee != nullptr) {
		collisionState = COLLISION_NEARBY;
		AddDeathDependence(lastCollidee, DEPENDENCE_LASTCOLWARN);
		return;
	}
}
