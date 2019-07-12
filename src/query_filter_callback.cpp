#include "pr_physx/query_filter_callback.hpp"
#include "pr_physx/environment.hpp"
#include "pr_physx/collision_object.hpp"

pragma::physics::PhysXQueryFilterCallback::PhysXQueryFilterCallback(ICollisionObject &collisionObject)
	: m_collisionObject{collisionObject}
{}

physx::PxQueryHitType::Enum pragma::physics::PhysXQueryFilterCallback::preFilter(
	const physx::PxFilterData& filterData, const physx::PxShape* shape, const physx::PxRigidActor* actor, physx::PxHitFlags& queryFlags)
{
	auto &controllerCollisionObj = m_collisionObject;
	if(actor == nullptr)
		return physx::PxQueryHitType::eBLOCK;
	auto *colObj = PhysXEnvironment::GetCollisionObject(*actor);
	auto mask = controllerCollisionObj.GetCollisionFilterGroup();
	auto group = colObj->GetCollisionFilterMask();
	return ((mask &group) != CollisionMask::None) ? physx::PxQueryHitType::eBLOCK : physx::PxQueryHitType::eNONE;
}

physx::PxQueryHitType::Enum pragma::physics::PhysXQueryFilterCallback::postFilter(const physx::PxFilterData& filterData, const physx::PxQueryHit& hit)
{
	return physx::PxQueryHitType::eBLOCK;
}
