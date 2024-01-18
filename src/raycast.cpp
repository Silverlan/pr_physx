/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include <cinttypes>
#include <limits>
#include <pragma/entities/entity_component_manager.hpp>
#include "pr_physx/raycast.hpp"
#include "pr_physx/environment.hpp"
#include "pr_physx/collision_object.hpp"
#include "pr_physx/shape.hpp"
#include <pragma/physics/raytraces.h>

pragma::physics::RayCastFilterCallback::RayCastFilterCallback(const pragma::physics::PhysXEnvironment &env, pragma::physics::IRayCastFilterCallback &rayCastFilterCallback, bool invertResult) : m_env {env}, m_rayCastFilterCallback {rayCastFilterCallback}, m_bInvertResult {invertResult} {}
physx::PxQueryHitType::Enum pragma::physics::RayCastFilterCallback::preFilter(const physx::PxFilterData &filterData, const physx::PxShape *shape, const physx::PxRigidActor *actor, physx::PxHitFlags &queryFlags) { return Filter(shape, actor, &IRayCastFilterCallback::PreFilter); }
physx::PxQueryHitType::Enum pragma::physics::RayCastFilterCallback::postFilter(const physx::PxFilterData &filterData, const physx::PxQueryHit &hit, const physx::PxShape *shape, const physx::PxRigidActor *actor) { return Filter(shape, actor, &IRayCastFilterCallback::PostFilter); }

physx::PxQueryHitType::Enum pragma::physics::RayCastFilterCallback::Filter(const physx::PxShape *shape, const physx::PxRigidActor *actor, RayCastHitType (IRayCastFilterCallback::*filter)(IShape &, IRigidBody &) const)
{
	if(shape == nullptr || shape->userData == nullptr || actor == nullptr || actor->userData == nullptr)
		return physx::PxQueryHitType::eNONE;
	auto *colObj = m_env.GetCollisionObject(*actor);
	if(colObj->IsRigid() == false)
		return physx::PxQueryHitType::eNONE;
	auto *rigidBody = m_env.GetCollisionObject(*actor)->GetRigidBody();
	if(rigidBody == nullptr)
		return physx::PxQueryHitType::eNONE;
	auto hitType = (m_rayCastFilterCallback.*filter)(m_env.GetShape(*shape)->GetShape(), *rigidBody);
	if(m_bInvertResult) {
		switch(hitType) {
		case RayCastHitType::None:
			hitType = RayCastHitType::Block;
			break;
		case RayCastHitType::Block:
			hitType = RayCastHitType::None;
			break;
		}
	}
	switch(hitType) {
	case RayCastHitType::None:
		return physx::PxQueryHitType::Enum::eNONE;
	case RayCastHitType::Touch:
		return physx::PxQueryHitType::Enum::eTOUCH;
	case RayCastHitType::Block:
	default:
		return physx::PxQueryHitType::Enum::eBLOCK;
	}
}
