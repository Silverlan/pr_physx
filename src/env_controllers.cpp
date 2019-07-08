#include "pr_physx/environment.hpp"
#include "pr_physx/material.hpp"
#include "pr_physx/shape.hpp"
#include "pr_physx/collision_object.hpp"
#include "pr_physx/controller.hpp"
#include <pragma/networkstate/networkstate.h>

void pragma::physics::PxEnvironment::InitializeControllerDesc(physx::PxControllerDesc &inOutDesc,float stepHeight,const Transform &startTransform)
{
	auto pos = ToPhysXVector(startTransform.GetOrigin());
	inOutDesc.contactOffset = 0.1f; // TODO: Scale
	inOutDesc.density = 10.f; // TODO
	inOutDesc.invisibleWallHeight = 0.f;
	inOutDesc.material = &dynamic_cast<PxMaterial&>(GetGenericMaterial()).GetInternalObject();
	inOutDesc.maxJumpHeight = 0.0;
	inOutDesc.nonWalkableMode = physx::PxControllerNonWalkableMode::ePREVENT_CLIMBING_AND_FORCE_SLIDING;
	inOutDesc.scaleCoeff = 0.8f;
	inOutDesc.slopeLimit = 1.f; // Arbitrary value > 0, to make sure slope limit is enabled (otherwise it cannot be enabled during runtime). Actual slope limit is set after character creation
	inOutDesc.stepOffset = ToPhysXLength(stepHeight);
	inOutDesc.position = physx::PxExtendedVec3{pos.x,pos.y,pos.z};
	inOutDesc.upDirection = ToPhysXNormal(uquat::up(startTransform.GetRotation()));
	inOutDesc.volumeGrowth = 1.5f;
}
util::TSharedHandle<pragma::physics::IController> pragma::physics::PxEnvironment::CreateController(PxUniquePtr<physx::PxController> c)
{
	auto *pActor = c->getActor();
	physx::PxShape *shapes;
	if(pActor == nullptr || pActor->getShapes(&shapes,1) == 0)
		return nullptr;
	auto &internalShape = shapes[0];
	std::shared_ptr<PxConvexShape> shape = nullptr;
	switch(internalShape.getGeometryType())
	{
	case physx::PxGeometryType::eBOX:
	{
		// Geometry and shape will automatically be removed by controller
		auto geometry = std::unique_ptr<physx::PxGeometry,void(*)(physx::PxGeometry*)>{&internalShape.getGeometry().box(),[](physx::PxGeometry*) {}};
		auto pxShape = std::unique_ptr<physx::PxShape,void(*)(physx::PxShape*)>{&internalShape,[](physx::PxShape*) {}};
		shape = CreateSharedPtr<PxConvexShape>(*this,std::move(pxShape),std::move(geometry));
		InitializeShape(*shape,true);
		break;
	}
	case physx::PxGeometryType::eCAPSULE:
	{
		// Geometry and shape will automatically be removed by controller
		auto geometry = std::unique_ptr<physx::PxGeometry,void(*)(physx::PxGeometry*)>{&internalShape.getGeometry().capsule(),[](physx::PxGeometry*) {}};
		auto pxShape = std::unique_ptr<physx::PxShape,void(*)(physx::PxShape*)>{&internalShape,[](physx::PxShape*) {}};
		shape = CreateSharedPtr<PxConvexShape>(*this,std::move(pxShape),std::move(geometry));
		InitializeShape(*shape,true);
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
	auto rigidBody = CreateSharedHandle<PxRigidDynamic>(*this,std::move(rigidDynamic),*shape,pRigidDynamic->getMass(),FromPhysXVector(pRigidDynamic->getMassSpaceInertiaTensor()));
	InitializeCollisionObject(*rigidBody);

	auto controller = util::shared_handle_cast<PxController,IController>(
		CreateSharedHandle<pragma::physics::PxController>(*this,std::move(c),util::shared_handle_cast<PxRigidDynamic,ICollisionObject>(rigidBody))
	);
	rigidBody->SetController(PxController::GetController(*controller));
	return controller;
}

util::TSharedHandle<pragma::physics::IController> pragma::physics::PxEnvironment::CreateCapsuleController(float halfWidth,float halfHeight,float stepHeight,umath::Degree slopeLimit,const Transform &startTransform)
{
	physx::PxCapsuleControllerDesc capsuleDesc {};
	InitializeControllerDesc(capsuleDesc,stepHeight,startTransform);
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
		pC->setUserData(&PxController::GetController(*controller));
		controller->SetSlopeLimit(slopeLimit);
	}
	AddController(*controller);
	return controller;
}
util::TSharedHandle<pragma::physics::IController> pragma::physics::PxEnvironment::CreateBoxController(const Vector3 &halfExtents,float stepHeight,umath::Degree slopeLimit,const Transform &startTransform)
{
	physx::PxBoxControllerDesc boxDesc {};
	InitializeControllerDesc(boxDesc,stepHeight,startTransform);
	boxDesc.halfHeight = halfExtents.y;
	boxDesc.halfSideExtent = halfExtents.z;
	boxDesc.halfForwardExtent = halfExtents.x;

	auto c = px_create_unique_ptr(m_controllerManager->createController(boxDesc));
	auto *pC = c.get();
	auto controller = c ? CreateController(std::move(c)) : nullptr;
	if(controller.IsValid())
	{
		pC->setUserData(&PxController::GetController(*controller));
		controller->SetSlopeLimit(slopeLimit);
	}
	AddController(*controller);
	return controller;
}
