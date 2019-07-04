#include "pr_physx/controller.hpp"
#include "pr_physx/environment.hpp"
#include "pr_physx/shape.hpp"
#include <pragma/networkstate/networkstate.h>

pragma::physics::PxController &pragma::physics::PxController::GetController(IController &c)
{
	return *static_cast<PxController*>(c.userData);
}
const pragma::physics::PxController &pragma::physics::PxController::GetController(const IController &o) {return GetController(const_cast<IController&>(o));}
pragma::physics::PxController::PxController(IEnvironment &env,PxUniquePtr<physx::PxController> controller,const util::TSharedHandle<ICollisionObject> &collisionObject)
	: IController{env,collisionObject},m_controller{std::move(controller)}
{
	userData = this;
}
pragma::physics::PxEnvironment &pragma::physics::PxController::GetPxEnv() const {return static_cast<PxEnvironment&>(m_physEnv);}
physx::PxController &pragma::physics::PxController::GetInternalObject() const {return *m_controller;}
pragma::physics::IController::CollisionFlags pragma::physics::PxController::DoMove(Vector3 &disp)
{
	//m_controller->getActor();
	physx::PxControllerFilters filters {};
	auto colFlags = GetInternalObject().move(
		GetPxEnv().ToPhysXVector(disp),
		0.f,
		GetPxEnv().GetNetworkState().DeltaTime(),
		filters
	);
	pragma::physics::IController::CollisionFlags flags = pragma::physics::IController::CollisionFlags::None;
	if(colFlags &physx::PxControllerCollisionFlag::eCOLLISION_DOWN)
		flags |= CollisionFlags::Down;
	if(colFlags &physx::PxControllerCollisionFlag::eCOLLISION_UP)
		flags |= CollisionFlags::Up;
	if(colFlags &physx::PxControllerCollisionFlag::eCOLLISION_SIDES)
		flags |= CollisionFlags::Sides;
	return flags;
}
Vector3 pragma::physics::PxController::GetDimensions() const
{
	physx::PxVec3 extents {0.f,0.f,0.f};
	switch(m_controller->getType())
	{
	case physx::PxControllerShapeType::eBOX:
	{
		auto &controller = static_cast<physx::PxBoxController&>(*m_controller);
		extents = {
			controller.getHalfForwardExtent(),
			controller.getHalfHeight(),
			controller.getHalfSideExtent()
		};
		break;
	}
	case physx::PxControllerShapeType::eCAPSULE:
	{
		auto &controller = static_cast<physx::PxCapsuleController&>(*m_controller);
		extents = {
			controller.getRadius(),
			controller.getHeight(),
			controller.getRadius()
		};
		break;
	}
	}
	return GetPxEnv().FromPhysXVector(extents);
}
void pragma::physics::PxController::SetDimensions(const Vector3 &dimensions)
{
	auto pxDim = GetPxEnv().ToPhysXVector(dimensions);
	switch(m_controller->getType())
	{
	case physx::PxControllerShapeType::eBOX:
	{
		auto &controller = static_cast<physx::PxBoxController&>(*m_controller);
		controller.setHalfForwardExtent(pxDim.x *0.5f);
		controller.setHalfHeight(pxDim.y *0.5f);
		controller.setHalfSideExtent(pxDim.z *0.5f);
		break;
	}
	case physx::PxControllerShapeType::eCAPSULE:
	{
		auto &controller = static_cast<physx::PxCapsuleController&>(*m_controller);
		controller.setHeight(pxDim.y *0.5f);
		controller.setRadius(pxDim.x *0.5f);
		break;
	}
	}
}
void pragma::physics::PxController::Resize(float newHeight)
{
	m_controller->resize(newHeight);
}
