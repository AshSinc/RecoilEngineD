/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */
#include "MoveTypeFactory.h"
#include "MoveDefHandler.h"
//#include "SpaceMoveType.h" //This needs to be the concrete class rather than abstract
#include "StrafeAirMoveType.h"
#include "HoverAirMoveType.h"
#include "HoverSpaceMoveType.h"
#include "StrafeSpaceMoveType.h"
#include "GroundMoveType.h"
#include "StaticMoveType.h"
#include "ScriptMoveType.h"

#include "Sim/Units/Unit.h"
#include "Sim/Units/UnitDef.h"

#include "System/Misc/TracyDefs.h"
#include <iostream>

void MoveTypeFactory::InitStatic() {
	RECOIL_DETAILED_TRACY_ZONE;
	static_assert(sizeof(CGroundMoveType) >= sizeof(CStrafeAirMoveType), "");
	static_assert(sizeof(CGroundMoveType) >= sizeof(CHoverAirMoveType ), "");
	static_assert(sizeof(CGroundMoveType) >= sizeof(CStaticMoveType   ), "");
	static_assert(sizeof(CGroundMoveType) >= sizeof(CScriptMoveType   ), "");
}

AMoveType* MoveTypeFactory::GetMoveType(CUnit* unit, const UnitDef* ud) {
	RECOIL_DETAILED_TRACY_ZONE;
	static_assert(sizeof(CGroundMoveType) <= sizeof(unit->amtMemBuffer), "");
	static_assert(sizeof(CScriptMoveType) <= sizeof(unit->smtMemBuffer), "");

	if (ud->IsGroundUnit()) {
		// mobile ground-unit
		assert(!ud->canfly);
		assert(ud->pathType != -1u);
		assert(unit->moveDef == nullptr);

		unit->moveDef = moveDefHandler.GetMoveDefByPathType(ud->pathType);

		return (new (unit->amtMemBuffer) CGroundMoveType(unit));
	}

	if (ud->IsSpaceUnit()) {
		if(ud->IsSpaceStrafingUnit())
			return (new (unit->amtMemBuffer) StrafeSpaceMoveType(unit)); //Need to add StrafeSpaceMoveType here enventually ISSUE - Actually I might not, HoverSpaceMoveType might be enough?
		else
			return (new (unit->amtMemBuffer) HoverSpaceMoveType(unit));
	}

	if (ud->IsAirUnit()) {
		// mobile air-unit
		assert(ud->canfly);
		assert(ud->pathType == -1u);
		assert(unit->moveDef == nullptr);

		if (ud->IsStrafingAirUnit())
			return (new (unit->amtMemBuffer) CStrafeAirMoveType(unit));

		// flying builders, transports, gunships
		return (new (unit->amtMemBuffer) CHoverAirMoveType(unit));
	}

	// unit is immobile (but not necessarily a structure)
	assert(ud->IsImmobileUnit());
	return (new (unit->amtMemBuffer) CStaticMoveType(unit));
}

AMoveType* MoveTypeFactory::GetScriptMoveType(CUnit* unit) {
	RECOIL_DETAILED_TRACY_ZONE;
	return (new (unit->smtMemBuffer) CScriptMoveType(unit));
}

