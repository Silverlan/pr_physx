/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "pr_physx/controller.hpp"
#include "pr_physx/environment.hpp"
#include "pr_physx/shape.hpp"
#include "pr_physx/material.hpp"
#include "pr_physx/collision_object.hpp"
#include "pr_physx/query_filter_callback.hpp"
#include <pragma/networkstate/networkstate.h>
#include <pragma/math/surfacematerial.h>

namespace pragma::physics
{
	class PhysXControllerFilterCallback
		: public physx::PxControllerFilterCallback
	{
	public:
		virtual bool filter(const physx::PxController& a, const physx::PxController& b) override
		{
			auto *controller0 = PhysXEnvironment::GetController(a);
			auto *co0 = controller0 ? controller0->GetCollisionObject() : nullptr;
			auto *controller1 = PhysXEnvironment::GetController(b);
			auto *co1 = controller1 ? controller1->GetCollisionObject() : nullptr;
			if(co0 == nullptr || co1 == nullptr)
				return true;
			auto mask = co0->GetCollisionFilterGroup();
			auto group = co1->GetCollisionFilterMask();
			return (mask &group) != CollisionMask::None;
		};
	};
};

pragma::physics::PhysXController &pragma::physics::PhysXController::GetController(IController &c)
{
	return *static_cast<PhysXController*>(c.GetUserData());
}
const pragma::physics::PhysXController &pragma::physics::PhysXController::GetController(const IController &o) {return GetController(const_cast<IController&>(o));}
static std::unique_ptr<pragma::physics::PhysXControllerFilterCallback> g_filterCallback = nullptr;
static uint32_t g_controllerCount = 0;
pragma::physics::PhysXController::PhysXController(IEnvironment &env,PhysXUniquePtr<physx::PxController> controller,const util::TSharedHandle<ICollisionObject> &collisionObject,const Vector3 &halfExtents,ShapeType shapeType)
	: IController{env,collisionObject,halfExtents,shapeType},m_controller{std::move(controller)}
{
	SetUserData(this);
	m_controller->getState(m_controllerState);
	if(g_controllerCount++ == 0)
		g_filterCallback = std::make_unique<PhysXControllerFilterCallback>();
}
pragma::physics::PhysXController::~PhysXController()
{
	if(--g_controllerCount == 0)
		g_filterCallback = nullptr;
}
void pragma::physics::PhysXController::Initialize()
{
	IController::Initialize();
	GetInternalObject().setUserData(this);
	m_queryFilterCallback = std::make_unique<PhysXQueryFilterCallback>(*GetCollisionObject());
}
void pragma::physics::PhysXController::RemoveWorldObject() {}
void pragma::physics::PhysXController::DoAddWorldObject() {}
pragma::physics::PhysXEnvironment &pragma::physics::PhysXController::GetPxEnv() const {return static_cast<PhysXEnvironment&>(m_physEnv);}
physx::PxController &pragma::physics::PhysXController::GetInternalObject() const {return *m_controller;}
pragma::physics::IShape *pragma::physics::PhysXController::GetGroundShape() const
{
	return (m_pGroundTouchingHit && m_pGroundTouchingHit->shape.expired() == false) ? &PhysXShape::GetShape(*m_pGroundTouchingHit->shape.lock()) : nullptr;
}
pragma::physics::IRigidBody *pragma::physics::PhysXController::GetGroundBody() const
{
	auto *colObj = (m_pGroundTouchingHit && m_pGroundTouchingHit->body.IsExpired() == false) ? &PhysXCollisionObject::GetCollisionObject(*m_pGroundTouchingHit->body.Get()) : nullptr;
	return (colObj && colObj->IsRigid()) ? colObj->GetRigidBody() : nullptr;
}
pragma::physics::IMaterial *pragma::physics::PhysXController::GetGroundMaterial() const
{
	return IsTouchingGround() ? m_pGroundTouchingHit->material.lock().get() : nullptr;
}
bool pragma::physics::PhysXController::IsTouchingGround() const {return m_pGroundTouchingHit != nullptr;}
std::optional<Vector3> pragma::physics::PhysXController::GetGroundTouchPos() const {return IsTouchingGround() ? m_pGroundTouchingHit->worldPos : std::optional<Vector3>{};}
std::optional<Vector3> pragma::physics::PhysXController::GetGroundTouchNormal() const {return IsTouchingGround() ? m_pGroundTouchingHit->worldNormal : std::optional<Vector3>{};}
pragma::physics::IController::CollisionFlags pragma::physics::PhysXController::GetCollisionFlags() const {return m_collisionFlags;}
void pragma::physics::PhysXController::MoveController(const Vector3 &displacement,bool testOnly)
{
	physx::PxFilterData filterData {0,0,0,0};
	physx::PxControllerFilters filters {};
	filters.mCCTFilterCallback = g_filterCallback.get();
	filters.mFilterCallback = m_queryFilterCallback.get();
	filters.mFilterData = &filterData; // Does this have to be copied?
	filters.mFilterFlags = physx::PxQueryFlag::eSTATIC | physx::PxQueryFlag::eDYNAMIC | physx::PxQueryFlag::ePREFILTER;
	m_controller->move(
		GetPxEnv().ToPhysXVector(displacement),
		0.f,
		GetPxEnv().GetNetworkState().DeltaTime(),
		filters
	);
	m_controller->getState(m_controllerState);
}
void pragma::physics::PhysXController::DoMove(Vector3 &disp)
{
	MoveController(disp,false);
}
void pragma::physics::PhysXController::SetPos(const Vector3 &pos)
{
	m_controller->setPosition(GetPxEnv().ToPhysXExtendedVector(pos));
}
Vector3 pragma::physics::PhysXController::GetPos() const
{
	return GetPxEnv().FromPhysXVector(m_controller->getPosition());
}
void pragma::physics::PhysXController::SetFootPos(const Vector3 &footPos)
{
	m_controller->setFootPosition(GetPxEnv().ToPhysXExtendedVector(footPos));
}
Vector3 pragma::physics::PhysXController::GetFootPos() const
{
	return GetPxEnv().FromPhysXVector(m_controller->getFootPosition());
}
void pragma::physics::PhysXController::SetUpDirection(const Vector3 &up)
{
	m_controller->setUpDirection(GetPxEnv().ToPhysXNormal(up));
}
Vector3 pragma::physics::PhysXController::GetUpDirection() const
{
	return GetPxEnv().FromPhysXNormal(m_controller->getUpDirection());
}
void pragma::physics::PhysXController::SetSlopeLimit(umath::Degree slopeLimit)
{
	m_controller->setSlopeLimit(umath::cos(umath::deg_to_rad(slopeLimit)));
}
umath::Degree pragma::physics::PhysXController::GetSlopeLimit() const
{
	return umath::rad_to_deg(umath::acos(m_controller->getSlopeLimit()));
}
void pragma::physics::PhysXController::SetStepHeight(float stepHeight)
{
	m_controller->setStepOffset(stepHeight);
}
float pragma::physics::PhysXController::GetStepHeight() const
{
	return m_controller->getStepOffset();
}
Vector3 pragma::physics::PhysXController::GetDimensions() const
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
void pragma::physics::PhysXController::SetDimensions(const Vector3 &dimensions)
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
void pragma::physics::PhysXController::Resize(float newHeight)
{
	m_controller->resize(newHeight);
}
Vector3 pragma::physics::PhysXController::GetLinearVelocity() const {return m_velocity;}
void pragma::physics::PhysXController::SetLinearVelocity(const Vector3 &vel) {m_velocity = vel;}
void pragma::physics::PhysXController::PreSimulate(float dt)
{
	m_preSimulationPosition = GetPxEnv().FromPhysXVector(m_controller->getPosition());
	auto &vel = const_cast<Vector3&>(GetMoveVelocity());
	m_touchingHits.clear();
	DoMove(vel);
}

void pragma::physics::PhysXController::PostSimulate(float dt)
{
	if(dt > 0.f)
	{
		auto deltaPos = GetPxEnv().FromPhysXVector(m_controller->getPosition()) -m_preSimulationPosition;
		if(m_controllerState.isMovingUp == false && uvec::dot(GetUpDirection(),deltaPos) > 0.f)
		{
			// Neutralize upwards velocity. This is primarily to make sure stepping
			// doesn't cause the controller to be launched upwards.
			deltaPos.y = 0.f;
		}
		m_velocity = deltaPos *(1.f /dt);
	}

	m_pGroundTouchingHit = nullptr;
	auto upDir = GetUpDirection();
	auto groundAng = umath::deg_to_rad(GetSlopeLimit());
	// Determine ground hit of controller.
	// m_controllerState.touchedActor is too unreliable, so we
	// have to determine it manually.
	for(auto &touchingHit : m_touchingHits)
	{
		auto ang = umath::acos(uvec::dot(touchingHit.worldNormal,upDir));
		if(ang > groundAng)
			continue;
		groundAng = ang;
		m_pGroundTouchingHit = &touchingHit;
	}
	
	m_collisionFlags = pragma::physics::IController::CollisionFlags::None;
	auto colFlags = m_controllerState.collisionFlags;
	if(colFlags &physx::PxControllerCollisionFlag::eCOLLISION_DOWN)
		m_collisionFlags |= CollisionFlags::Down;
	if(colFlags &physx::PxControllerCollisionFlag::eCOLLISION_UP)
		m_collisionFlags |= CollisionFlags::Up;
	if(colFlags &physx::PxControllerCollisionFlag::eCOLLISION_SIDES)
		m_collisionFlags |= CollisionFlags::Sides;

	// Reset controller state
	m_controllerState = {};
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
	auto *controller = hit.controller ? PhysXEnvironment::GetController(*hit.controller) : nullptr;
	if(controller == nullptr)
		return;
	controller->m_touchingHits.push_back({});
	auto &touchingHit = controller->m_touchingHits.back();
	auto *shape = hit.shape ? PhysXEnvironment::GetShape(*hit.shape) : nullptr;
	touchingHit.shape = shape ? std::static_pointer_cast<IShape>(shape->GetShape().shared_from_this()) : std::weak_ptr<pragma::physics::IShape>{};

	auto *actor = hit.actor ? PhysXEnvironment::GetCollisionObject(*hit.actor) : nullptr;
	touchingHit.body = actor ? util::weak_shared_handle_cast<IBase,IRigidBody>(actor->GetHandle()) : util::TWeakSharedHandle<pragma::physics::IRigidBody>{};

	touchingHit.worldNormal = controller->GetPxEnv().FromPhysXNormal(hit.worldNormal);
	touchingHit.worldPos = controller->GetPxEnv().FromPhysXVector(hit.worldPos);
	
	auto *material = hit.shape ? hit.shape->getMaterialFromInternalFaceIndex(hit.triangleIndex) : nullptr;
	auto *pxMaterial = material ? PhysXEnvironment::GetMaterial(*material) : nullptr;
	if(material == nullptr && shape)
	{
		auto *surfMat = shape->GetShape().GetSurfaceMaterial();
		pxMaterial = surfMat ? &PhysXMaterial::GetMaterial(surfMat->GetPhysicsMaterial()) : nullptr;
	}
	touchingHit.material = pxMaterial ? std::static_pointer_cast<IMaterial>(pxMaterial->shared_from_this()) : std::weak_ptr<pragma::physics::IMaterial>{};
}

void pragma::physics::CustomUserControllerHitReport::onControllerHit(const physx::PxControllersHit& hit) {}

void pragma::physics::CustomUserControllerHitReport::onObstacleHit(const physx::PxControllerObstacleHit& hit) {}
