#include "pr_physx/controller.hpp"
#include "pr_physx/environment.hpp"
#include "pr_physx/shape.hpp"
#include "pr_physx/material.hpp"
#include "pr_physx/collision_object.hpp"
#include <pragma/networkstate/networkstate.h>
#include <pragma/math/surfacematerial.h>

#pragma optimize("",off)
pragma::physics::PxController &pragma::physics::PxController::GetController(IController &c)
{
	return *static_cast<PxController*>(c.GetUserData());
}
const pragma::physics::PxController &pragma::physics::PxController::GetController(const IController &o) {return GetController(const_cast<IController&>(o));}
pragma::physics::PxController::PxController(IEnvironment &env,PxUniquePtr<physx::PxController> controller,const util::TSharedHandle<ICollisionObject> &collisionObject)
	: IController{env,collisionObject},m_controller{std::move(controller)}
{
	SetUserData(this);
	m_controller->getState(m_controllerState);
}
void pragma::physics::PxController::Initialize()
{
	IController::Initialize();
	GetInternalObject().setUserData(this);
}
void pragma::physics::PxController::RemoveWorldObject() {}
void pragma::physics::PxController::DoAddWorldObject() {}
pragma::physics::PxEnvironment &pragma::physics::PxController::GetPxEnv() const {return static_cast<PxEnvironment&>(m_physEnv);}
physx::PxController &pragma::physics::PxController::GetInternalObject() const {return *m_controller;}
pragma::physics::IShape *pragma::physics::PxController::GetGroundShape() const
{
	return (m_pGroundTouchingHit && m_pGroundTouchingHit->shape.expired() == false) ? &PxShape::GetShape(*m_pGroundTouchingHit->shape.lock()) : nullptr;
}
pragma::physics::IRigidBody *pragma::physics::PxController::GetGroundBody() const
{
	auto *colObj = (m_pGroundTouchingHit && m_pGroundTouchingHit->body.IsExpired() == false) ? &PxCollisionObject::GetCollisionObject(*m_pGroundTouchingHit->body.Get()) : nullptr;
	return (colObj && colObj->IsRigid()) ? colObj->GetRigidBody() : nullptr;
}
pragma::physics::IMaterial *pragma::physics::PxController::GetGroundMaterial() const
{
	return IsTouchingGround() ? m_pGroundTouchingHit->material.lock().get() : nullptr;
}
bool pragma::physics::PxController::IsTouchingGround() const {return m_pGroundTouchingHit != nullptr;}
std::optional<Vector3> pragma::physics::PxController::GetGroundTouchPos() const {return IsTouchingGround() ? m_pGroundTouchingHit->worldPos : std::optional<Vector3>{};}
std::optional<Vector3> pragma::physics::PxController::GetGroundTouchNormal() const {return IsTouchingGround() ? m_pGroundTouchingHit->worldNormal : std::optional<Vector3>{};}
pragma::physics::IController::CollisionFlags pragma::physics::PxController::GetCollisionFlags() const
{
	pragma::physics::IController::CollisionFlags flags = pragma::physics::IController::CollisionFlags::None;
	auto colFlags = m_controllerState.collisionFlags;
	if(colFlags &physx::PxControllerCollisionFlag::eCOLLISION_DOWN)
		flags |= CollisionFlags::Down;
	if(colFlags &physx::PxControllerCollisionFlag::eCOLLISION_UP)
		flags |= CollisionFlags::Up;
	if(colFlags &physx::PxControllerCollisionFlag::eCOLLISION_SIDES)
		flags |= CollisionFlags::Sides;
	return flags;
}
void pragma::physics::PxController::DoMove(Vector3 &disp)
{
	physx::PxControllerFilters filters {};
	auto colFlags = GetInternalObject().move(
		GetPxEnv().ToPhysXVector(disp),
		0.f,
		GetPxEnv().GetNetworkState().DeltaTime(),
		filters
	);
	m_controller->getState(m_controllerState);
}
void pragma::physics::PxController::SetPos(const Vector3 &pos)
{
	m_controller->setPosition(GetPxEnv().ToPhysXExtendedVector(pos));
}
Vector3 pragma::physics::PxController::GetPos() const
{
	return GetPxEnv().FromPhysXVector(m_controller->getPosition());
}
void pragma::physics::PxController::SetFootPos(const Vector3 &footPos)
{
	m_controller->setFootPosition(GetPxEnv().ToPhysXExtendedVector(footPos));
}
Vector3 pragma::physics::PxController::GetFootPos() const
{
	return GetPxEnv().FromPhysXVector(m_controller->getFootPosition());
}
void pragma::physics::PxController::SetUpDirection(const Vector3 &up)
{
	m_controller->setUpDirection(GetPxEnv().ToPhysXNormal(up));
}
Vector3 pragma::physics::PxController::GetUpDirection() const
{
	return GetPxEnv().FromPhysXNormal(m_controller->getUpDirection());
}
void pragma::physics::PxController::SetSlopeLimit(umath::Degree slopeLimit)
{
	m_controller->setSlopeLimit(umath::cos(umath::deg_to_rad(slopeLimit)));
}
umath::Degree pragma::physics::PxController::GetSlopeLimit() const
{
	return umath::rad_to_deg(umath::acos(m_controller->getSlopeLimit()));
}
void pragma::physics::PxController::SetStepHeight(float stepHeight)
{
	m_controller->setStepOffset(stepHeight);
}
float pragma::physics::PxController::GetStepHeight() const
{
	return m_controller->getStepOffset();
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
	auto footPos = m_controller->getFootPosition();
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
	m_controller->setFootPosition(footPos);
}
void pragma::physics::PxController::Resize(float newHeight)
{
	m_controller->resize(newHeight);
}
void pragma::physics::PxController::PreSimulate()
{
	auto &vel = const_cast<Vector3&>(GetMoveVelocity());
	m_touchingHits.clear();
	DoMove(vel);
}
void pragma::physics::PxController::PostSimulate()
{
	// 'touchedActor' should be the actor the controller is standing on,
	// but for some reason it's NULL in most cases, even if the controller is
	// standing on something.
	m_pGroundTouchingHit = nullptr;
	if(m_controllerState.touchedActor == nullptr)
		return;
	auto it = std::find_if(m_touchingHits.begin(),m_touchingHits.end(),[this](const TouchingHit &touchingHit) {
		return m_controllerState.touchedActor == &PxCollisionObject::GetCollisionObject(*touchingHit.body).GetInternalObject();
	});
	if(it == m_touchingHits.end())
		return;
	m_pGroundTouchingHit = &*it;
}

//////////////////////

physx::PxControllerBehaviorFlags pragma::physics::CustomControllerBehaviorCallback::getBehaviorFlags(const physx::PxShape& shape, const physx::PxActor& actor)
{
	return physx::PxControllerBehaviorFlag::eCCT_CAN_RIDE_ON_OBJECT;
}

physx::PxControllerBehaviorFlags pragma::physics::CustomControllerBehaviorCallback::getBehaviorFlags(const physx::PxController& controller)
{
	return physx::PxControllerBehaviorFlag::eCCT_CAN_RIDE_ON_OBJECT;
}

physx::PxControllerBehaviorFlags pragma::physics::CustomControllerBehaviorCallback::getBehaviorFlags(const physx::PxObstacle& obstacle)
{
	return physx::PxControllerBehaviorFlag::eCCT_CAN_RIDE_ON_OBJECT;
}

//////////////////////

void pragma::physics::CustomUserControllerHitReport::onShapeHit(const physx::PxControllerShapeHit& hit)
{
	auto *controller = hit.controller ? PxEnvironment::GetController(*hit.controller) : nullptr;
	if(controller == nullptr)
		return;
	controller->m_touchingHits.push_back({});
	auto &touchingHit = controller->m_touchingHits.back();
	auto *shape = hit.shape ? PxEnvironment::GetShape(*hit.shape) : nullptr;
	touchingHit.shape = shape ? std::static_pointer_cast<IShape>(shape->shared_from_this()) : std::weak_ptr<pragma::physics::IShape>{};

	auto *actor = hit.actor ? PxEnvironment::GetCollisionObject(*hit.actor) : nullptr;
	touchingHit.body = actor ? util::weak_shared_handle_cast<IBase,IRigidBody>(actor->GetHandle()) : util::TWeakSharedHandle<pragma::physics::IRigidBody>{};

	touchingHit.worldNormal = controller->GetPxEnv().FromPhysXNormal(hit.worldNormal);
	touchingHit.worldPos = controller->GetPxEnv().FromPhysXVector(hit.worldPos);
	
	auto *material = hit.shape ? hit.shape->getMaterialFromInternalFaceIndex(hit.triangleIndex) : nullptr;
	auto *pxMaterial = material ? PxEnvironment::GetMaterial(*material) : nullptr;
	if(material == nullptr)
	{
		auto *surfMat = shape->GetSurfaceMaterial();
		pxMaterial = surfMat ? &PxMaterial::GetMaterial(surfMat->GetPhysicsMaterial()) : nullptr;
	}
	touchingHit.material = pxMaterial ? std::static_pointer_cast<IMaterial>(pxMaterial->shared_from_this()) : std::weak_ptr<pragma::physics::IMaterial>{};
}

void pragma::physics::CustomUserControllerHitReport::onControllerHit(const physx::PxControllersHit& hit) {}

void pragma::physics::CustomUserControllerHitReport::onObstacleHit(const physx::PxControllerObstacleHit& hit) {}
#pragma optimize("",on)
