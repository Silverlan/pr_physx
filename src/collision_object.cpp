/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "pr_physx/collision_object.hpp"
#include "pr_physx/environment.hpp"
#include "pr_physx/shape.hpp"
#include "pr_physx/material.hpp"
#include "pr_physx/controller.hpp"
#include <extensions/PxRigidBodyExt.h>
#include <pragma/util/util_game.hpp>

pragma::physics::PhysXCollisionObject &pragma::physics::PhysXCollisionObject::GetCollisionObject(ICollisionObject &o) { return *static_cast<PhysXCollisionObject *>(o.GetUserData()); }
const pragma::physics::PhysXCollisionObject &pragma::physics::PhysXCollisionObject::GetCollisionObject(const ICollisionObject &o) { return GetCollisionObject(const_cast<ICollisionObject &>(o)); }
pragma::physics::PhysXCollisionObject::PhysXCollisionObject(IEnvironment &env, PhysXUniquePtr<physx::PxActor> actor, IShape &shape) : ICollisionObject {env, shape}, m_actor {std::move(actor)}, m_actorShapeCollection {*this} { SetUserData(this); }
void pragma::physics::PhysXCollisionObject::Initialize()
{
	ICollisionObject::Initialize();
	GetInternalObject().userData = this;
}
void pragma::physics::PhysXCollisionObject::OnRemove()
{
	ApplyCollisionShape(nullptr);
	ICollisionObject::OnRemove();
}
physx::PxActor &pragma::physics::PhysXCollisionObject::GetInternalObject() const { return *m_actor; }
pragma::physics::PhysXEnvironment &pragma::physics::PhysXCollisionObject::GetPxEnv() const { return static_cast<PhysXEnvironment &>(m_physEnv); }
void pragma::physics::PhysXCollisionObject::GetAABB(Vector3 &min, Vector3 &max) const
{
	auto bounds = m_actor->getWorldBounds();
	min = GetPxEnv().FromPhysXVector(bounds.minimum);
	max = GetPxEnv().FromPhysXVector(bounds.maximum);
}
void pragma::physics::PhysXCollisionObject::SetSleepReportEnabled(bool reportEnabled) { m_actor->setActorFlag(physx::PxActorFlag::eSEND_SLEEP_NOTIFIES, reportEnabled); }
bool pragma::physics::PhysXCollisionObject::IsSleepReportEnabled() const { return m_actor->getActorFlags().isSet(physx::PxActorFlag::eSEND_SLEEP_NOTIFIES); }

void pragma::physics::PhysXCollisionObject::SetTrigger(bool bTrigger) { m_actorShapeCollection.SetTrigger(bTrigger); }
bool pragma::physics::PhysXCollisionObject::IsTrigger() const { return m_actorShapeCollection.IsTrigger(); }

void pragma::physics::PhysXCollisionObject::TransformLocalPose(const umath::Transform &t) { m_actorShapeCollection.TransformLocalPose(t); }
pragma::physics::NoCollisionCategoryId pragma::physics::PhysXCollisionObject::DisableSelfCollisions()
{
	if(m_noCollisionCategory == nullptr)
		m_noCollisionCategory = GetPxEnv().GetUniqueNoCollisionCategory();
	for(auto &actorShape : m_actorShapeCollection.GetActorShapes()) {
		auto &pxActorShape = actorShape->GetActorShape();

		auto queryFilterData = pxActorShape.getQueryFilterData();
		queryFilterData.word3 = *m_noCollisionCategory;
		pxActorShape.setQueryFilterData(queryFilterData);

		auto simFilterData = pxActorShape.getSimulationFilterData();
		simFilterData.word3 = *m_noCollisionCategory;
		pxActorShape.setSimulationFilterData(simFilterData);
	}
	return *m_noCollisionCategory;
}
pragma::physics::PhysXActorShapeCollection &pragma::physics::PhysXCollisionObject::GetActorShapeCollection() const { return m_actorShapeCollection; }
void pragma::physics::PhysXCollisionObject::RemoveWorldObject()
{
	if(m_actor == nullptr || IsSpawned() == false)
		return;
	if(IsAwake())
		OnSleep();
	GetPxEnv().GetScene().removeActor(*m_actor);
	m_actor = nullptr;
}
void pragma::physics::PhysXCollisionObject::DoAddWorldObject() { GetPxEnv().GetScene().addActor(*m_actor); }

//////////////////

pragma::physics::PhysXRigidBody::PhysXRigidBody(IEnvironment &env, PhysXUniquePtr<physx::PxActor> actor, IShape &shape) : PhysXCollisionObject {env, std::move(actor), shape}, IRigidBody {env, shape}, ICollisionObject {env, shape} {}
physx::PxRigidActor &pragma::physics::PhysXRigidBody::GetInternalObject() const { return static_cast<physx::PxRigidDynamic &>(PhysXCollisionObject::GetInternalObject()); }

void pragma::physics::PhysXRigidBody::SetContactProcessingThreshold(float threshold)
{
	// GetInternalObject().setContactReportThreshold();
}
Vector3 pragma::physics::PhysXRigidBody::GetPos() const
{
	auto *pController = GetController();
	if(pController)
		return pController->GetPos();
	return GetPxEnv().FromPhysXVector(GetInternalObject().getGlobalPose().p);
}
void pragma::physics::PhysXRigidBody::SetPos(const Vector3 &pos)
{
	auto *pController = GetController();
	if(pController) {
		// If this rigid body belongs to a controller, we mustn't
		// update its position directly! Instead, the controller
		// has to be teleported.
		pController->SetPos(pos);
		return;
	}
	auto pose = GetInternalObject().getGlobalPose();
	pose.p = GetPxEnv().ToPhysXVector(pos);
	GetInternalObject().setGlobalPose(pose);
}
Quat pragma::physics::PhysXRigidBody::GetRotation() const
{
	auto *pController = GetController();
	if(pController) {
		// Controller has no rotation
		return uquat::identity();
	}
	return GetPxEnv().FromPhysXRotation(GetInternalObject().getGlobalPose().q);
}
void pragma::physics::PhysXRigidBody::SetRotation(const Quat &rot)
{
	auto *pController = GetController();
	if(pController) {
		// Controller mustn't be rotated
		return;
	}
	auto pose = GetInternalObject().getGlobalPose();
	pose.q = GetPxEnv().ToPhysXRotation(rot);
	GetInternalObject().setGlobalPose(pose);
}
umath::Transform pragma::physics::PhysXRigidBody::GetWorldTransform()
{
	auto t = GetInternalObject().getGlobalPose();
	return GetPxEnv().CreateTransform(t);
}
void pragma::physics::PhysXRigidBody::SetWorldTransform(const umath::Transform &t)
{
	auto *pController = GetController();
	if(pController) {
		// Only the position of the transform
		// can be applied to a controller
		SetPos(t.GetOrigin());
		return;
	}
	auto pxPose = GetPxEnv().CreatePxTransform(t);
	GetInternalObject().setGlobalPose(pxPose);
}

void pragma::physics::PhysXRigidBody::SetSimulationEnabled(bool b) { GetInternalObject().setActorFlag(physx::PxActorFlag::eDISABLE_SIMULATION, !b); }
bool pragma::physics::PhysXRigidBody::IsSimulationEnabled() const { return GetInternalObject().getActorFlags().isSet(physx::PxActorFlag::eDISABLE_SIMULATION); }
void pragma::physics::PhysXRigidBody::SetCollisionsEnabled(bool enabled)
{
	// TODO
}
void pragma::physics::PhysXRigidBody::RemoveWorldObject()
{
	if(m_controller.GetRawPtr() != nullptr)
		return; // Controller manages lifetime of rigid body
	PhysXCollisionObject::RemoveWorldObject();
}
void pragma::physics::PhysXRigidBody::DoAddWorldObject()
{
	if(m_controller.GetRawPtr() != nullptr)
		return; // Controller manages lifetime of rigid body
	PhysXCollisionObject::DoAddWorldObject();
}

void pragma::physics::PhysXRigidBody::ApplyCollisionShape(pragma::physics::IShape *optShape)
{
	if(m_controller.GetRawPtr())
		return; // If this is a controller, we mustn't detach or attach any shapes
	auto &o = GetInternalObject();
	// Clear all current shapes
	auto numShapes = o.getNbShapes();
	std::vector<physx::PxShape *> shapes {numShapes};
	numShapes = o.getShapes(shapes.data(), numShapes);
	for(auto i = decltype(numShapes) {0u}; i < numShapes; ++i) {
		auto *pShape = shapes.at(i);
		o.detachShape(*pShape);
	}
	//
	m_actorShapeCollection.Clear();
	if(optShape == nullptr || o.getNbShapes() > 0)
		return;
	if(optShape->IsCompoundShape() == false) {
		auto *mat = PhysXShape::GetShape(*optShape).GetMaterial();
		if(mat == nullptr)
			mat = &GetPxEnv().GetGenericMaterial();
		m_actorShapeCollection.AttachShapeToActor(PhysXShape::GetShape(*optShape), PhysXMaterial::GetMaterial(*mat));
	}
	else {
		auto &compoundShape = static_cast<PhysXCompoundShape &>(PhysXShape::GetShape(*optShape));
		auto &subShapes = compoundShape.GetShapes();
		std::vector<physx::PxShape *> pxShapes {};
		pxShapes.reserve(subShapes.size());
		auto parentPose = compoundShape.GetLocalPose();
		for(auto i = decltype(subShapes.size()) {0u}; i < subShapes.size(); ++i) {
			auto &shapeInfo = subShapes.at(i);
			if(shapeInfo.shape->IsCompoundShape() == true)
				continue; // Compound shapes of compound shapes currently not supported
			auto *mat = PhysXShape::GetShape(*shapeInfo.shape).GetMaterial();
			if(mat == nullptr) {
				mat = compoundShape.GetMaterial();
				if(mat == nullptr)
					mat = &GetPxEnv().GetGenericMaterial();
			}
			m_actorShapeCollection.AttachShapeToActor(PhysXShape::GetShape(*shapeInfo.shape), PhysXMaterial::GetMaterial(*mat), parentPose * shapeInfo.localPose);
		}
	}

	// We need to re-apply our collision group and mask
	SetCollisionFilterGroup(GetCollisionFilterGroup());
	SetCollisionFilterMask(GetCollisionFilterMask());
	if(m_noCollisionCategory)
		DisableSelfCollisions();
}
void pragma::physics::PhysXRigidBody::DoSetCollisionFilterGroup(CollisionMask group)
{
	for(auto &actorShape : GetActorShapeCollection().GetActorShapes()) {
		auto &pxActorShape = actorShape->GetActorShape();

		auto queryFilterData = pxActorShape.getQueryFilterData();
		queryFilterData.word0 = umath::to_integral(group);
		pxActorShape.setQueryFilterData(queryFilterData);

		auto simFilterData = pxActorShape.getSimulationFilterData();
		simFilterData.word0 = umath::to_integral(group);
		pxActorShape.setSimulationFilterData(simFilterData);
	}
}
void pragma::physics::PhysXRigidBody::DoSetCollisionFilterMask(CollisionMask mask)
{
	for(auto &actorShape : GetActorShapeCollection().GetActorShapes()) {
		auto &pxActorShape = actorShape->GetActorShape();

		auto queryFilterData = pxActorShape.getQueryFilterData();
		queryFilterData.word1 = umath::to_integral(mask);
		pxActorShape.setQueryFilterData(queryFilterData);

		auto simFilterData = pxActorShape.getSimulationFilterData();
		simFilterData.word1 = umath::to_integral(mask);
		pxActorShape.setSimulationFilterData(simFilterData);
	}
}
umath::Transform pragma::physics::PhysXRigidBody::GetBaseTransform() { return GetPxEnv().CreateTransform(GetInternalObject().getGlobalPose()); }
void pragma::physics::PhysXRigidBody::SetBaseTransform(const umath::Transform &t) { GetInternalObject().setGlobalPose(GetPxEnv().CreatePxTransform(t)); }
void pragma::physics::PhysXRigidBody::SetCenterOfMassOffset(const Vector3 &offset)
{
	// TODO
}
Vector3 pragma::physics::PhysXRigidBody::GetCenterOfMassOffset() const
{
	// TODO
	return Vector3 {};
}
void pragma::physics::PhysXRigidBody::SetMassProps(float mass, const Vector3 &inertia)
{
	// TODO
}
void pragma::physics::PhysXRigidBody::SetController(PhysXController &controller) { m_controller = util::weak_shared_handle_cast<IBase, PhysXController>(controller.GetHandle()); }
pragma::physics::PhysXController *pragma::physics::PhysXRigidBody::GetController() const { return m_controller.Get(); }

//////////////////

pragma::physics::PhysXRigidDynamic::PhysXRigidDynamic(IEnvironment &env, PhysXUniquePtr<physx::PxActor> actor, IShape &shape) : PhysXRigidBody {env, std::move(actor), shape}, ICollisionObject {env, shape}, IRigidBody {env, shape} {}
physx::PxRigidDynamic &pragma::physics::PhysXRigidDynamic::GetInternalObject() const { return static_cast<physx::PxRigidDynamic &>(PhysXRigidBody::GetInternalObject()); }
void pragma::physics::PhysXRigidDynamic::SetActivationState(ActivationState state)
{
	switch(state) {
	case ActivationState::Asleep:
	case ActivationState::WaitForDeactivation:
		GetInternalObject().putToSleep();
		break;
	case ActivationState::Active:
	case ActivationState::AlwaysActive:
		GetInternalObject().wakeUp();
		break;
	}
	// TODO
	//GetInternalObject().setSleepThreshold
}
pragma::physics::ICollisionObject::ActivationState pragma::physics::PhysXRigidDynamic::GetActivationState() const
{
	if(GetInternalObject().isSleeping())
		return ActivationState::Asleep;
	return ActivationState::Active;
}

void pragma::physics::PhysXRigidDynamic::ApplyForce(const Vector3 &force, bool autoWake)
{
	if(IsKinematic())
		return;
	GetInternalObject().addForce(GetPxEnv().ToPhysXVector(force), physx::PxForceMode::eFORCE, autoWake);
}
void pragma::physics::PhysXRigidDynamic::ApplyForce(const Vector3 &force, const Vector3 &relPos, bool autoWake)
{
	if(IsKinematic())
		return;
	physx::PxRigidBodyExt::addForceAtLocalPos(GetInternalObject(), GetPxEnv().ToPhysXVector(force), GetPxEnv().ToPhysXVector(relPos), physx::PxForceMode::eFORCE, autoWake);
}
void pragma::physics::PhysXRigidDynamic::ApplyImpulse(const Vector3 &impulse, bool autoWake)
{
	if(IsKinematic())
		return;
	GetInternalObject().addForce(GetPxEnv().ToPhysXVector(impulse), physx::PxForceMode::eIMPULSE, autoWake);
}
void pragma::physics::PhysXRigidDynamic::ApplyImpulse(const Vector3 &impulse, const Vector3 &relPos, bool autoWake)
{
	if(IsKinematic())
		return;
	physx::PxRigidBodyExt::addForceAtLocalPos(GetInternalObject(), GetPxEnv().ToPhysXVector(impulse), GetPxEnv().ToPhysXVector(relPos), physx::PxForceMode::eIMPULSE, autoWake);
}
void pragma::physics::PhysXRigidDynamic::ApplyTorque(const Vector3 &torque, bool autoWake)
{
	if(IsKinematic())
		return;
	GetInternalObject().addTorque(GetPxEnv().ToPhysXTorque(torque), physx::PxForceMode::eFORCE, autoWake);
}
void pragma::physics::PhysXRigidDynamic::ApplyTorqueImpulse(const Vector3 &torque, bool autoWake)
{
	if(IsKinematic())
		return;
	GetInternalObject().addTorque(GetPxEnv().ToPhysXTorque(torque), physx::PxForceMode::eIMPULSE, autoWake);
}
void pragma::physics::PhysXRigidDynamic::ClearForces()
{
	GetInternalObject().clearForce(physx::PxForceMode::eACCELERATION);
	GetInternalObject().clearForce(physx::PxForceMode::eVELOCITY_CHANGE);
	GetInternalObject().clearTorque(physx::PxForceMode::eACCELERATION);
	GetInternalObject().clearTorque(physx::PxForceMode::eVELOCITY_CHANGE);
}
Vector3 pragma::physics::PhysXRigidDynamic::GetTotalForce() const
{
	// TODO
	return Vector3 {};
}
Vector3 pragma::physics::PhysXRigidDynamic::GetTotalTorque() const
{
	// TODO
	return Vector3 {};
}
float pragma::physics::PhysXRigidDynamic::GetMass() const { return GetInternalObject().getMass(); }
void pragma::physics::PhysXRigidDynamic::SetMass(float mass) { GetInternalObject().setMass(mass); }
void pragma::physics::PhysXRigidDynamic::SetMassAndUpdateInertia(float mass) { physx::PxRigidBodyExt::setMassAndUpdateInertia(static_cast<physx::PxRigidBody &>(GetInternalObject()), mass); }
Vector3 pragma::physics::PhysXRigidDynamic::GetInertia()
{
	auto inertiaTensor = static_cast<physx::PxRigidBody &>(GetInternalObject()).getMassSpaceInertiaTensor();
	return GetPxEnv().FromPhysXVector(inertiaTensor * umath::pow2(pragma::units_to_metres(1.f)));
}
Mat3 pragma::physics::PhysXRigidDynamic::GetInvInertiaTensorWorld() const
{
	// TODO
	//auto inertiaTensor = static_cast<physx::PxRigidBody&>(GetInternalObject()).getMassSpaceInvInertiaTensor();
	return Mat3 {};
}
void pragma::physics::PhysXRigidDynamic::SetInertia(const Vector3 &inertia)
{
	auto inertiaTensor = GetPxEnv().ToPhysXVector(inertia) / umath::pow2(pragma::units_to_metres(1.f));
	static_cast<physx::PxRigidBody &>(GetInternalObject()).setMassSpaceInertiaTensor(inertiaTensor);
}
Vector3 pragma::physics::PhysXRigidDynamic::GetCenterOfMass() const { return GetPxEnv().FromPhysXVector(GetInternalObject().getCMassLocalPose().p); }
Vector3 pragma::physics::PhysXRigidDynamic::GetLinearVelocity() const { return GetPxEnv().FromPhysXVector(GetInternalObject().getLinearVelocity()); }
Vector3 pragma::physics::PhysXRigidDynamic::GetAngularVelocity() const { return GetPxEnv().FromPhysXVector(GetInternalObject().getAngularVelocity()); }
void pragma::physics::PhysXRigidDynamic::SetLinearVelocity(const Vector3 &vel, bool autoWake)
{
	if(IsKinematic())
		return;
	GetInternalObject().setLinearVelocity(GetPxEnv().ToPhysXVector(vel), autoWake);
}
void pragma::physics::PhysXRigidDynamic::SetAngularVelocity(const Vector3 &vel, bool autoWake)
{
	if(IsKinematic())
		return;
	GetInternalObject().setAngularVelocity(GetPxEnv().ToPhysXVector(vel), autoWake);
}
void pragma::physics::PhysXRigidDynamic::SetLinearFactor(const Vector3 &factor)
{
	// TODO
}
void pragma::physics::PhysXRigidDynamic::SetAngularFactor(const Vector3 &factor)
{
	// TODO
}
Vector3 pragma::physics::PhysXRigidDynamic::GetLinearFactor() const
{
	// TODO
	return Vector3 {};
}
Vector3 pragma::physics::PhysXRigidDynamic::GetAngularFactor() const
{
	// TODO
	return Vector3 {};
}
void pragma::physics::PhysXRigidDynamic::SetLinearDamping(float damping) { GetInternalObject().setLinearDamping(damping); }
void pragma::physics::PhysXRigidDynamic::SetAngularDamping(float damping) { GetInternalObject().setAngularDamping(damping); }
float pragma::physics::PhysXRigidDynamic::GetLinearDamping() const { return GetInternalObject().getLinearDamping(); }
float pragma::physics::PhysXRigidDynamic::GetAngularDamping() const { return GetInternalObject().getAngularDamping(); }
void pragma::physics::PhysXRigidDynamic::SetLinearSleepingThreshold(float threshold) { GetInternalObject().setSleepThreshold(threshold); }
void pragma::physics::PhysXRigidDynamic::SetAngularSleepingThreshold(float threshold)
{
	// Not available in PhysX
}
float pragma::physics::PhysXRigidDynamic::GetLinearSleepingThreshold() const { return GetInternalObject().getSleepThreshold(); }
float pragma::physics::PhysXRigidDynamic::GetAngularSleepingThreshold() const { return GetInternalObject().getSleepThreshold(); }
void pragma::physics::PhysXRigidDynamic::SetCenterOfMassOffset(const Vector3 &offset)
{
	auto pose = GetInternalObject().getCMassLocalPose();
	pose.p = GetPxEnv().ToPhysXVector(offset);
	GetInternalObject().setCMassLocalPose(pose);
}
Vector3 pragma::physics::PhysXRigidDynamic::GetCenterOfMassOffset() const
{
	auto pose = GetInternalObject().getCMassLocalPose();
	return GetPxEnv().FromPhysXVector(pose.p);
}
void pragma::physics::PhysXRigidDynamic::SetKinematic(bool bKinematic) { GetInternalObject().setRigidBodyFlag(physx::PxRigidBodyFlag::eKINEMATIC, bKinematic); }
bool pragma::physics::PhysXRigidDynamic::IsKinematic() const { return GetInternalObject().getRigidBodyFlags().isSet(physx::PxRigidBodyFlag::eKINEMATIC); }
void pragma::physics::PhysXRigidDynamic::WakeUp(bool forceActivation) { GetInternalObject().wakeUp(); }
void pragma::physics::PhysXRigidDynamic::PutToSleep() { GetInternalObject().putToSleep(); }
bool pragma::physics::PhysXRigidDynamic::IsStatic() const { return GetInternalObject().getRigidBodyFlags().isSet(physx::PxRigidBodyFlag::eKINEMATIC); }
void pragma::physics::PhysXRigidDynamic::SetStatic(bool b)
{
	// Dynamic rigid bodies cannot be transformed into static rigid bodies in PhysX,
	// so we'll just turn it into a kinematic object instead.
	if(b)
		SetCCDEnabled(false); // CCD is not supported for kinematic objects
	GetInternalObject().setRigidBodyFlag(physx::PxRigidBodyFlag::eKINEMATIC, b);
}
void pragma::physics::PhysXRigidDynamic::SetCCDEnabled(bool b) { GetInternalObject().setRigidBodyFlag(physx::PxRigidBodyFlag::eENABLE_CCD, b); }
void pragma::physics::PhysXRigidDynamic::ApplyCollisionShape(pragma::physics::IShape *optShape)
{
	PhysXRigidBody::ApplyCollisionShape(optShape);
	if(optShape == nullptr)
		return;
	auto &o = GetInternalObject();
	physx::PxRigidBodyExt::setMassAndUpdateInertia(o, GetMass(), nullptr, IsTrigger());
}

//////////////////

pragma::physics::PhysXRigidStatic::PhysXRigidStatic(IEnvironment &env, PhysXUniquePtr<physx::PxActor> actor, IShape &shape) : PhysXRigidBody {env, std::move(actor), shape}, ICollisionObject {env, shape}, IRigidBody {env, shape} {}
physx::PxRigidStatic &pragma::physics::PhysXRigidStatic::GetInternalObject() const { return static_cast<physx::PxRigidStatic &>(PhysXRigidBody::GetInternalObject()); }
void pragma::physics::PhysXRigidStatic::SetActivationState(ActivationState state) {}
pragma::physics::PhysXRigidBody::ActivationState pragma::physics::PhysXRigidStatic::GetActivationState() const { return PhysXRigidBody::ActivationState::Asleep; }
bool pragma::physics::PhysXRigidStatic::IsStatic() const { return true; }
void pragma::physics::PhysXRigidStatic::SetStatic(bool b) {}
void pragma::physics::PhysXRigidStatic::WakeUp(bool forceActivation) {}
void pragma::physics::PhysXRigidStatic::PutToSleep() {}

void pragma::physics::PhysXRigidStatic::SetCCDEnabled(bool b) {}
void pragma::physics::PhysXRigidStatic::ApplyForce(const Vector3 &force, bool autoWake) {}
void pragma::physics::PhysXRigidStatic::ApplyForce(const Vector3 &force, const Vector3 &relPos, bool autoWake) {}
void pragma::physics::PhysXRigidStatic::ApplyImpulse(const Vector3 &impulse, bool autoWake) {}
void pragma::physics::PhysXRigidStatic::ApplyImpulse(const Vector3 &impulse, const Vector3 &relPos, bool autoWake) {}
void pragma::physics::PhysXRigidStatic::ApplyTorque(const Vector3 &torque, bool autoWake) {}
void pragma::physics::PhysXRigidStatic::ApplyTorqueImpulse(const Vector3 &torque, bool autoWake) {}
void pragma::physics::PhysXRigidStatic::ClearForces() {}
Vector3 pragma::physics::PhysXRigidStatic::GetTotalForce() const { return Vector3 {}; }
Vector3 pragma::physics::PhysXRigidStatic::GetTotalTorque() const { return Vector3 {}; }
float pragma::physics::PhysXRigidStatic::GetMass() const { return 0.f; }
void pragma::physics::PhysXRigidStatic::SetMass(float mass) {}
void pragma::physics::PhysXRigidStatic::SetMassAndUpdateInertia(float mass) {}
Vector3 pragma::physics::PhysXRigidStatic::GetInertia() { return Vector3 {}; }
Mat3 pragma::physics::PhysXRigidStatic::GetInvInertiaTensorWorld() const { return Mat3 {}; }
void pragma::physics::PhysXRigidStatic::SetInertia(const Vector3 &inertia) {}
Vector3 pragma::physics::PhysXRigidStatic::GetCenterOfMass() const { return Vector3 {}; }
Vector3 pragma::physics::PhysXRigidStatic::GetLinearVelocity() const { return Vector3 {}; }
Vector3 pragma::physics::PhysXRigidStatic::GetAngularVelocity() const { return Vector3 {}; }
void pragma::physics::PhysXRigidStatic::SetLinearVelocity(const Vector3 &vel, bool autoWake) {}
void pragma::physics::PhysXRigidStatic::SetAngularVelocity(const Vector3 &vel, bool autoWake) {}
void pragma::physics::PhysXRigidStatic::SetLinearFactor(const Vector3 &factor) {}
void pragma::physics::PhysXRigidStatic::SetAngularFactor(const Vector3 &factor) {}
Vector3 pragma::physics::PhysXRigidStatic::GetLinearFactor() const { return Vector3 {}; }
Vector3 pragma::physics::PhysXRigidStatic::GetAngularFactor() const { return Vector3 {}; }
void pragma::physics::PhysXRigidStatic::SetLinearDamping(float damping) {}
void pragma::physics::PhysXRigidStatic::SetAngularDamping(float damping) {}
float pragma::physics::PhysXRigidStatic::GetLinearDamping() const { return 0.f; }
float pragma::physics::PhysXRigidStatic::GetAngularDamping() const { return 0.f; }
void pragma::physics::PhysXRigidStatic::SetLinearSleepingThreshold(float threshold) {}
void pragma::physics::PhysXRigidStatic::SetAngularSleepingThreshold(float threshold) {}
float pragma::physics::PhysXRigidStatic::GetLinearSleepingThreshold() const { return 0.f; }
float pragma::physics::PhysXRigidStatic::GetAngularSleepingThreshold() const { return 0.f; }

void pragma::physics::PhysXRigidStatic::SetKinematic(bool bKinematic) {}
bool pragma::physics::PhysXRigidStatic::IsKinematic() const { return false; }

//////////////////

pragma::physics::PhysXSoftBody::PhysXSoftBody(IEnvironment &env, PhysXUniquePtr<physx::PxActor> actor, IShape &shape) : ISoftBody {env, shape, {}}, PhysXCollisionObject {env, std::move(actor), shape} {}
physx::PxActor &pragma::physics::PhysXSoftBody::GetInternalObject() const { return PhysXCollisionObject::GetInternalObject(); }
