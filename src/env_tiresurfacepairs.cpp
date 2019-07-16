#include "pr_physx/environment.hpp"
#include "pr_physx/material.hpp"
#include <pragma/networkstate/networkstate.h>
#include <pragma/game/game.h>

#pragma optimize("",off)
void pragma::physics::PhysXEnvironment::UpdateSurfaceTypes()
{
	auto &tireTypeManager = GetTireTypeManager();
	auto &tireTypes = tireTypeManager.GetRegisteredTypes();
	auto *game = GetNetworkState().GetGameState();
	auto &surfMats = game->GetSurfaceMaterials();

	std::vector<physx::PxVehicleDrivableSurfaceType> surfacesTypes {};
	std::vector<const physx::PxMaterial*> physMats {};
	surfacesTypes.reserve(surfMats.size());
	physMats.reserve(surfMats.size());
	for(auto &surfMat : surfMats)
	{
		auto *surfType = surfMat.GetSurfaceType();
		if(surfType == nullptr)
			continue;
		surfacesTypes.push_back({});
		surfacesTypes.back().mType = surfType->GetId();

		physMats.push_back(&pragma::physics::PhysXMaterial::GetMaterial(surfMat.GetPhysicsMaterial()).GetInternalObject());
	}

	m_surfaceTirePairs = {
		physx::PxVehicleDrivableSurfaceToTireFrictionPairs::allocate(tireTypes.size(),surfacesTypes.size()),
		[](physx::PxVehicleDrivableSurfaceToTireFrictionPairs *ptr) {
			ptr->release();
		}
	};

	m_surfaceTirePairs->setup(tireTypes.size(),surfacesTypes.size(),physMats.data(),surfacesTypes.data());

	for(auto &hTireType : tireTypes)
	{
		if(hTireType.IsExpired())
			continue;
		auto tireTypeId = hTireType->GetId();
		for(auto &pair : hTireType->GetFrictionModifiers())
			m_surfaceTirePairs->setTypePairFriction(pair.first->GetId(),tireTypeId,pair.second);
	}
}
#pragma optimize("",on)
