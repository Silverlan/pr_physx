#include "pr_physx/environment.hpp"
#include "pr_physx/material.hpp"
#include "pr_physx/shape.hpp"
#include "pr_physx/collision_object.hpp"
#include "pr_physx/controller.hpp"
#include <pragma/networkstate/networkstate.h>

void pragma::physics::PhysXEnvironment::InitializeControllerDesc(physx::PxControllerDesc &inOutDesc,float halfHeight,float stepHeight,const Transform &startTransform)
{
	auto pos = ToPhysXVector(startTransform.GetOrigin());
	inOutDesc.contactOffset = 0.1f *0.025f; // TODO: Scale
	inOutDesc.density = 10.f; // TODO
	inOutDesc.invisibleWallHeight = 0.f;
	inOutDesc.material = &dynamic_cast<PhysXMaterial&>(GetGenericMaterial()).GetInternalObject();
	inOutDesc.maxJumpHeight = 0.0;
	inOutDesc.nonWalkableMode = physx::PxControllerNonWalkableMode::ePREVENT_CLIMBING; // ePREVENT_CLIMBING_AND_FORCE_SLIDING
	inOutDesc.scaleCoeff = 0.8f;
	inOutDesc.slopeLimit = 1.f; // Arbitrary value > 0, to make sure slope limit is enabled (otherwise it cannot be enabled during runtime). Actual slope limit is set after character creation
	inOutDesc.stepOffset = ToPhysXLength(stepHeight);
	inOutDesc.position = physx::PxExtendedVec3{pos.x,pos.y +halfHeight,pos.z};
	inOutDesc.upDirection = ToPhysXNormal(uquat::up(startTransform.GetRotation()));
	inOutDesc.volumeGrowth = 1.5f;
}
util::TSharedHandle<pragma::physics::IController> pragma::physics::PhysXEnvironment::CreateController(PhysXUniquePtr<physx::PxController> c)
{
	auto *pActor = c->getActor();
	physx::PxShape *shapes;
	if(pActor == nullptr || pActor->getShapes(&shapes,1) == 0)
		return nullptr;
	auto &internalShape = shapes[0];
	std::shared_ptr<PhysXConvexShape> shape = nullptr;
	switch(internalShape.getGeometryType())
	{
	case physx::PxGeometryType::eBOX:
	{
		// Geometry and shape will automatically be removed by controller
		auto geometry = std::shared_ptr<physx::PxGeometry>{&internalShape.getGeometry().box(),[](physx::PxGeometry*) {}};
		shape = CreateSharedPtr<PhysXConvexShape>(*this,geometry);
		break;
	}
	case physx::PxGeometryType::eCAPSULE:
	{
		// Geometry and shape will automatically be removed by controller
		auto geometry = std::shared_ptr<physx::PxGeometry>{&internalShape.getGeometry().capsule(),[](physx::PxGeometry*) {}};
		shape = CreateSharedPtr<PhysXConvexShape>(*this,geometry);
		break;
	}
	default:
		return nullptr;
	}
	if(shape == nullptr)
		return nullptr;

	// Actor will automatically be removed by controller
	auto rigidDynamic = std::unique_ptr<physx::PxActor,void(*)(physx::PxActor*)>{pActor,[](physx::PxActor*) {}};
	auto *pRigidDynamic = static_cast<physx::PxRigidDynamic*>(rigidDynamic.get());
	auto rigidBody = CreateSharedHandle<PhysXRigidDynamic>(*this,std::move(rigidDynamic),*shape);
	rigidBody->GetActorShapeCollection().AddShape(*shape,internalShape);
	InitializeCollisionObject(*rigidBody);

	auto controller = util::shared_handle_cast<PhysXController,IController>(
		CreateSharedHandle<pragma::physics::PhysXController>(*this,std::move(c),util::shared_handle_cast<PhysXRigidDynamic,ICollisionObject>(rigidBody))
	);
	rigidBody->SetController(PhysXController::GetController(*controller));
	return controller;
}

util::TSharedHandle<pragma::physics::IController> pragma::physics::PhysXEnvironment::CreateCapsuleController(float halfWidth,float halfHeight,float stepHeight,umath::Degree slopeLimit,const Transform &startTransform)
{
	physx::PxCapsuleControllerDesc capsuleDesc {};
	InitializeControllerDesc(capsuleDesc,halfHeight,stepHeight,startTransform);
	capsuleDesc.climbingMode = physx::PxCapsuleClimbingMode::eEASY;
	capsuleDesc.height = halfHeight *2.f;
	capsuleDesc.radius = halfWidth;
	capsuleDesc.behaviorCallback = m_controllerBehaviorCallback.get();
	capsuleDesc.reportCallback = m_controllerHitReport.get();

	auto c = px_create_unique_ptr(m_controllerManager->createController(capsuleDesc));
	auto *pC = c.get();
	auto controller = c ? CreateController(std::move(c)) : nullptr;
	if(controller.IsValid())
	{
		pC->setUserData(&PhysXController::GetController(*controller));
		controller->SetSlopeLimit(slopeLimit);
	}
	AddController(*controller);
	return controller;
}
util::TSharedHandle<pragma::physics::IController> pragma::physics::PhysXEnvironment::CreateBoxController(const Vector3 &halfExtents,float stepHeight,umath::Degree slopeLimit,const Transform &startTransform)
{
	physx::PxBoxControllerDesc boxDesc {};
	InitializeControllerDesc(boxDesc,halfExtents.y,stepHeight,startTransform);
	boxDesc.halfHeight = halfExtents.y;
	boxDesc.halfSideExtent = halfExtents.z;
	boxDesc.halfForwardExtent = halfExtents.x;

	auto c = px_create_unique_ptr(m_controllerManager->createController(boxDesc));
	auto *pC = c.get();
	auto controller = c ? CreateController(std::move(c)) : nullptr;
	if(controller.IsValid())
	{
		pC->setUserData(&PhysXController::GetController(*controller));
		controller->SetSlopeLimit(slopeLimit);
	}
	AddController(*controller);
	return controller;
}
