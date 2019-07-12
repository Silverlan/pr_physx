#include "pr_physx/sim_event_callback.hpp"
#include "pr_physx/environment.hpp"
#include "pr_physx/shape.hpp"
#include "pr_physx/material.hpp"
#include "pr_physx/constraint.hpp"
#include "pr_physx/collision_object.hpp"
#include <pragma/physics/contact.hpp>

void pragma::physics::PhysXSimulationEventCallback::onConstraintBreak(physx::PxConstraintInfo *constraints,physx::PxU32 count)
{
	for(auto i=decltype(count){0u};i<count;++i)
	{
		auto &constraintInfo = constraints[i];
		auto *pJoint = reinterpret_cast<physx::PxJoint*>(constraintInfo.externalReference);
		auto *joint = pJoint ? PhysXEnvironment::GetConstraint(*pJoint) : nullptr;
		if(joint == nullptr)
			continue;
		joint->OnBroken();
	}
}

void pragma::physics::PhysXSimulationEventCallback::onWake(physx::PxActor **actors, physx::PxU32 count)
{
	for(auto i=decltype(count){0u};i<count;++i)
	{
		auto *pActor = dynamic_cast<physx::PxRigidActor*>(actors[i]);
		auto *actor = pActor ? PhysXEnvironment::GetCollisionObject(*pActor) : nullptr;
		if(actor == nullptr || actor->IsSleepReportEnabled() == false)
			continue;
		actor->OnWake();
	}
}

void pragma::physics::PhysXSimulationEventCallback::onSleep(physx::PxActor **actors, physx::PxU32 count)
{
	for(auto i=decltype(count){0u};i<count;++i)
	{
		auto *pActor = dynamic_cast<physx::PxRigidActor*>(actors[i]);
		auto *actor = pActor ? PhysXEnvironment::GetCollisionObject(*pActor) : nullptr;
		if(actor == nullptr || actor->IsSleepReportEnabled() == false)
			continue;
		actor->OnSleep();
	}
}

void pragma::physics::PhysXSimulationEventCallback::onContact(const physx::PxContactPairHeader &pairHeader, const physx::PxContactPair *pairs, physx::PxU32 nbPairs)
{
	if(pairHeader.flags &(physx::PxContactPairHeaderFlag::eREMOVED_ACTOR_0 | physx::PxContactPairHeaderFlag::eREMOVED_ACTOR_1))
		return;
	auto *actor0 = pairHeader.actors[0] ? PhysXEnvironment::GetCollisionObject(*pairHeader.actors[0]) : nullptr;
	if(actor0->IsContactReportEnabled() == false)
		return;
	auto *actor1 = pairHeader.actors[1] ? PhysXEnvironment::GetCollisionObject(*pairHeader.actors[1]) : nullptr;
	if(actor0 == nullptr || actor1 == nullptr)
		return;
	auto &pxEnv = actor0->GetPxEnv();
	for(auto i=decltype(nbPairs){0u};i<nbPairs;++i)
	{
		auto &contactPair = pairs[i];
		if(contactPair.flags &(physx::PxContactPairFlag::eREMOVED_SHAPE_0 | physx::PxContactPairFlag::eREMOVED_SHAPE_1))
			continue;
		ContactInfo contactInfo {};
		contactInfo.flags = ContactInfo::Flags::None;
		if(contactPair.flags &physx::PxContactPairFlag::eACTOR_PAIR_HAS_FIRST_TOUCH)
			contactInfo.flags |= ContactInfo::Flags::StartTouch;
		if(contactPair.flags &physx::PxContactPairFlag::eACTOR_PAIR_LOST_TOUCH)
			contactInfo.flags |= ContactInfo::Flags::EndTouch;
		contactInfo.shape0 = contactPair.shapes[0] ? std::static_pointer_cast<IShape>(PhysXEnvironment::GetShape(*contactPair.shapes[0])->shared_from_this()) : nullptr;
		contactInfo.shape1 = contactPair.shapes[1] ? std::static_pointer_cast<IShape>(PhysXEnvironment::GetShape(*contactPair.shapes[1])->shared_from_this()) : nullptr;
		contactInfo.collisionObj0 = util::weak_shared_handle_cast<IBase,ICollisionObject>(actor0->GetHandle());
		contactInfo.collisionObj1 = util::weak_shared_handle_cast<IBase,ICollisionObject>(actor1->GetHandle());
		std::vector<physx::PxContactPairPoint> contactPoints {contactPair.contactCount};
		auto numContacts = contactPair.extractContacts(contactPoints.data(),contactPoints.size());
		contactPoints.resize(numContacts);
		contactInfo.contactPoints.reserve(numContacts);
		for(auto &cp : contactPoints)
		{
			contactInfo.contactPoints.push_back({});
			auto &contactPoint = contactInfo.contactPoints.back();
			contactPoint.impulse = pxEnv.FromPhysXVector(cp.impulse);
			contactPoint.normal = pxEnv.FromPhysXNormal(cp.normal);
			contactPoint.position = pxEnv.FromPhysXVector(cp.position);
			contactPoint.distance = pxEnv.FromPhysXLength(cp.separation);

			auto *mat0 = contactPair.shapes[0]->getMaterialFromInternalFaceIndex(cp.internalFaceIndex0);
			contactPoint.material0 = mat0 ? std::static_pointer_cast<IMaterial>(PhysXEnvironment::GetMaterial(*mat0)->shared_from_this()) : nullptr;

			auto *mat1 = contactPair.shapes[1]->getMaterialFromInternalFaceIndex(cp.internalFaceIndex1);
			contactPoint.material1 = mat1 ? std::static_pointer_cast<IMaterial>(PhysXEnvironment::GetMaterial(*mat1)->shared_from_this()) : nullptr;
		}
		actor0->OnContact(contactInfo);

		if(umath::is_flag_set(contactInfo.flags,ContactInfo::Flags::StartTouch))
			actor0->OnStartTouch(*actor1);
		if(umath::is_flag_set(contactInfo.flags,ContactInfo::Flags::EndTouch))
			actor0->OnEndTouch(*actor1);
	}
}

void pragma::physics::PhysXSimulationEventCallback::onTrigger(physx::PxTriggerPair *pairs, physx::PxU32 count)
{
	for(auto i=decltype(count){0u};i<count;++i)
	{
		auto &triggerPair = pairs[i];
		if(triggerPair.flags &(physx::PxTriggerPairFlag::eREMOVED_SHAPE_TRIGGER | physx::PxTriggerPairFlag::eREMOVED_SHAPE_OTHER))
			continue;
		auto *triggerActor = triggerPair.triggerActor ? PhysXEnvironment::GetCollisionObject(*triggerPair.triggerActor) : nullptr;
		auto *otherActor = triggerPair.otherActor ? PhysXEnvironment::GetCollisionObject(*triggerPair.otherActor) : nullptr;
		if(triggerActor == nullptr || otherActor == nullptr)
			continue;
		if(triggerPair.status &physx::PxPairFlag::eNOTIFY_TOUCH_FOUND)
			triggerActor->OnStartTouch(*otherActor);
		if(triggerPair.status &physx::PxPairFlag::eNOTIFY_TOUCH_LOST)
			triggerActor->OnEndTouch(*otherActor);
	}
}

void pragma::physics::PhysXSimulationEventCallback::onAdvance(const physx::PxRigidBody * const *bodyBuffer,const physx::PxTransform *poseBuffer,const physx::PxU32 count)
{

}

pragma::physics::PhysXSimulationEventCallback::~PhysXSimulationEventCallback() {}
