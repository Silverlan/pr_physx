#include "pr_physx/collision_object.hpp"
#include "pr_physx/environment.hpp"
#include "pr_physx/shape.hpp"
#include <extensions/PxRigidBodyExt.h>

#pragma optimize("",off)
pragma::physics::PxCollisionObject &pragma::physics::PxCollisionObject::GetCollisionObject(ICollisionObject &o)
{
	return *static_cast<PxCollisionObject*>(o.userData);
}
const pragma::physics::PxCollisionObject &pragma::physics::PxCollisionObject::GetCollisionObject(const ICollisionObject &o) {return GetCollisionObject(const_cast<ICollisionObject&>(o));}
pragma::physics::PxCollisionObject::PxCollisionObject(IEnvironment &env,PxUniquePtr<physx::PxActor> actor,IShape &shape)
	: ICollisionObject{env,shape},m_actor{std::move(actor)}
{
	userData = this;
}
physx::PxActor &pragma::physics::PxCollisionObject::GetInternalObject() const {return *m_actor;}
pragma::physics::PxEnvironment &pragma::physics::PxCollisionObject::GetPxEnv() const {return static_cast<PxEnvironment&>(m_physEnv);}
void pragma::physics::PxCollisionObject::GetAABB(Vector3 &min,Vector3 &max) const
{
	auto bounds = m_actor->getWorldBounds();
	min = GetPxEnv().FromPhysXVector(bounds.minimum);
	max = GetPxEnv().FromPhysXVector(bounds.maximum);
}

void pragma::physics::PxCollisionObject::RemoveWorldObject()
{
	GetPxEnv().GetScene().removeActor(*m_actor);
}
void pragma::physics::PxCollisionObject::DoAddWorldObject()
{
	GetPxEnv().GetScene().addActor(*m_actor);
}

//////////////////

pragma::physics::PxRigidBody::PxRigidBody(IEnvironment &env,PxUniquePtr<physx::PxRigidDynamic> actor,IShape &shape,float mass,const Vector3 &localInertia)
	: PxCollisionObject{env,px_cast_unique_ptr<physx::PxRigidDynamic,physx::PxActor>(std::move(actor)),shape},IRigidBody{env,mass,shape,localInertia},ICollisionObject{env,shape}
{}
physx::PxRigidDynamic &pragma::physics::PxRigidBody::GetInternalObject() const {return static_cast<physx::PxRigidDynamic&>(PxCollisionObject::GetInternalObject());}
void pragma::physics::PxRigidBody::SetActivationState(ActivationState state)
{
	switch(state)
	{
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
pragma::physics::ICollisionObject::ActivationState pragma::physics::PxRigidBody::GetActivationState() const
{
	if(GetInternalObject().isSleeping())
		return ActivationState::Asleep;
	return ActivationState::Active;
}

void pragma::physics::PxRigidBody::SetContactProcessingThreshold(float threshold)
{
	// GetInternalObject().setContactReportThreshold();
}
bool pragma::physics::PxRigidBody::IsStatic() const
{
	return GetInternalObject().getRigidBodyFlags().isSet(physx::PxRigidBodyFlag::eKINEMATIC);
}
void pragma::physics::PxRigidBody::SetStatic(bool b)
{
	// Dynamic rigid bodies cannot be transformed into static rigid bodies in PhysX,
	// so we'll just turn it into a kinematic object instead.
	GetInternalObject().setRigidBodyFlag(physx::PxRigidBodyFlag::eKINEMATIC,b);
}

void pragma::physics::PxRigidBody::SetCCDEnabled(bool b)
{
	GetInternalObject().setRigidBodyFlag(physx::PxRigidBodyFlag::eENABLE_CCD,b);
}
Vector3 pragma::physics::PxRigidBody::GetPos() const
{
	return GetPxEnv().FromPhysXVector(GetInternalObject().getGlobalPose().p);
}
void pragma::physics::PxRigidBody::SetPos(const Vector3 &pos)
{
	auto pose = GetInternalObject().getGlobalPose();
	pose.p = GetPxEnv().ToPhysXVector(pos);
	GetInternalObject().setGlobalPose(pose);
}
Quat pragma::physics::PxRigidBody::GetRotation() const
{
	return GetPxEnv().FromPhysXRotation(GetInternalObject().getGlobalPose().q);
}
void pragma::physics::PxRigidBody::SetRotation(const Quat &rot)
{
	auto pose = GetInternalObject().getGlobalPose();
	pose.q = GetPxEnv().ToPhysXRotation(rot);
	GetInternalObject().setGlobalPose(pose);
}
pragma::physics::Transform pragma::physics::PxRigidBody::GetWorldTransform()
{
	auto t = GetInternalObject().getGlobalPose();
	return GetPxEnv().CreateTransform(t);
}
void pragma::physics::PxRigidBody::SetWorldTransform(const Transform &t)
{
	auto pxPose = GetPxEnv().CreatePxTransform(t);
	GetInternalObject().setGlobalPose(pxPose);
}
void pragma::physics::PxRigidBody::WakeUp(bool forceActivation)
{
	GetInternalObject().wakeUp();
}
void pragma::physics::PxRigidBody::PutToSleep()
{
	GetInternalObject().putToSleep();
}
bool pragma::physics::PxRigidBody::IsAsleep() const
{
	return GetInternalObject().isSleeping();
}

bool pragma::physics::PxRigidBody::IsTrigger()
{
	auto *pShape = GetCollisionShape();
	return pShape ? pShape->IsTrigger() : false;
}

void pragma::physics::PxRigidBody::SetSimulationEnabled(bool b)
{
	GetInternalObject().setActorFlag(physx::PxActorFlag::eDISABLE_SIMULATION,!b);
}
bool pragma::physics::PxRigidBody::IsSimulationEnabled() const
{
	return GetInternalObject().getActorFlags().isSet(physx::PxActorFlag::eDISABLE_SIMULATION);
}
void pragma::physics::PxRigidBody::SetCollisionsEnabled(bool enabled)
{
	// TODO
}

void pragma::physics::PxRigidBody::ApplyCollisionShape(pragma::physics::IShape *optShape)
{
	auto &o = GetInternalObject();
	// Clear all current shapes
	auto numShapes = o.getNbShapes();
	std::vector<physx::PxShape*> shapes {numShapes};
	numShapes = o.getShapes(shapes.data(),numShapes);
	for(auto i=decltype(numShapes){0u};i<numShapes;++i)
	{
		auto *pShape = shapes.at(i);
		o.detachShape(*pShape);
	}
	//

	o.attachShape(dynamic_cast<PxShape*>(optShape)->GetInternalObject());
	physx::PxRigidBodyExt::updateMassAndInertia(o,optShape->GetDensity(),nullptr,optShape->IsTrigger());
}
void pragma::physics::PxRigidBody::DoSetCollisionFilterGroup(CollisionMask group)
{
	// TODO
}
void pragma::physics::PxRigidBody::DoSetCollisionFilterMask(CollisionMask mask)
{
	// TODO
}

//

void pragma::physics::PxRigidBody::ApplyForce(const Vector3 &force)
{
	GetInternalObject().addForce(GetPxEnv().ToPhysXVector(force),physx::PxForceMode::eFORCE);
}
void pragma::physics::PxRigidBody::ApplyForce(const Vector3 &force,const Vector3 &relPos)
{
	physx::PxRigidBodyExt::addForceAtLocalPos(GetInternalObject(),GetPxEnv().ToPhysXVector(force),GetPxEnv().ToPhysXVector(relPos),physx::PxForceMode::eFORCE);
}
void pragma::physics::PxRigidBody::ApplyImpulse(const Vector3 &impulse)
{
	GetInternalObject().addForce(GetPxEnv().ToPhysXVector(impulse),physx::PxForceMode::eIMPULSE);
}
void pragma::physics::PxRigidBody::ApplyImpulse(const Vector3 &impulse,const Vector3 &relPos)
{
	physx::PxRigidBodyExt::addForceAtLocalPos(GetInternalObject(),GetPxEnv().ToPhysXVector(impulse),GetPxEnv().ToPhysXVector(relPos),physx::PxForceMode::eIMPULSE);
}
void pragma::physics::PxRigidBody::ApplyTorque(const Vector3 &torque)
{
	GetInternalObject().addTorque(GetPxEnv().ToPhysXTorque(torque),physx::PxForceMode::eFORCE);
}
void pragma::physics::PxRigidBody::ApplyTorqueImpulse(const Vector3 &torque)
{
	GetInternalObject().addTorque(GetPxEnv().ToPhysXTorque(torque),physx::PxForceMode::eIMPULSE);
}
void pragma::physics::PxRigidBody::ClearForces()
{
	GetInternalObject().clearForce(physx::PxForceMode::eACCELERATION);
	GetInternalObject().clearForce(physx::PxForceMode::eVELOCITY_CHANGE);
	GetInternalObject().clearTorque(physx::PxForceMode::eACCELERATION);
	GetInternalObject().clearTorque(physx::PxForceMode::eVELOCITY_CHANGE);
}
Vector3 pragma::physics::PxRigidBody::GetTotalForce()
{
	// TODO
	return Vector3{};
}
Vector3 pragma::physics::PxRigidBody::GetTotalTorque()
{
	// TODO
	return Vector3{};
}
void pragma::physics::PxRigidBody::SetMassProps(float mass,const Vector3 &inertia)
{
	// TODO
}
float pragma::physics::PxRigidBody::GetMass() const
{
	return GetInternalObject().getMass();
}
void pragma::physics::PxRigidBody::SetMass(float mass)
{
	GetInternalObject().setMass(mass);
}
Vector3 &pragma::physics::PxRigidBody::GetInertia()
{
	// TODO
	return Vector3{};
}
Mat3 pragma::physics::PxRigidBody::GetInvInertiaTensorWorld() const
{
	// TODO
	return Mat3{};
}
void pragma::physics::PxRigidBody::SetInertia(const Vector3 &inertia)
{
	// TODO
}
Vector3 pragma::physics::PxRigidBody::GetLinearVelocity() const
{
	return GetPxEnv().FromPhysXVector(GetInternalObject().getLinearVelocity());
}
Vector3 pragma::physics::PxRigidBody::GetAngularVelocity() const
{
	return GetPxEnv().FromPhysXVector(GetInternalObject().getAngularVelocity());
}
void pragma::physics::PxRigidBody::SetLinearVelocity(const Vector3 &vel)
{
	GetInternalObject().setLinearVelocity(GetPxEnv().ToPhysXVector(vel));
}
void pragma::physics::PxRigidBody::SetAngularVelocity(const Vector3 &vel)
{
	GetInternalObject().setAngularVelocity(GetPxEnv().ToPhysXVector(vel));
}
void pragma::physics::PxRigidBody::SetLinearFactor(const Vector3 &factor)
{
	// TODO
}
void pragma::physics::PxRigidBody::SetAngularFactor(const Vector3 &factor)
{
	// TODO
}
Vector3 pragma::physics::PxRigidBody::GetLinearFactor() const
{
	// TODO
	return Vector3{};
}
Vector3 pragma::physics::PxRigidBody::GetAngularFactor() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PxRigidBody::SetLinearDamping(float damping)
{
	GetInternalObject().setLinearDamping(damping);
}
void pragma::physics::PxRigidBody::SetAngularDamping(float damping)
{
	GetInternalObject().setAngularDamping(damping);
}
float pragma::physics::PxRigidBody::GetLinearDamping() const
{
	return GetInternalObject().getLinearDamping();
}
float pragma::physics::PxRigidBody::GetAngularDamping() const
{
	return GetInternalObject().getAngularDamping();
}
void pragma::physics::PxRigidBody::SetLinearSleepingThreshold(float threshold)
{
	GetInternalObject().setSleepThreshold(threshold);
}
void pragma::physics::PxRigidBody::SetAngularSleepingThreshold(float threshold)
{
	// Not available in PhysX
}
float pragma::physics::PxRigidBody::GetLinearSleepingThreshold() const
{
	return GetInternalObject().getSleepThreshold();
}
float pragma::physics::PxRigidBody::GetAngularSleepingThreshold() const
{
	return GetInternalObject().getSleepThreshold();
}
void pragma::physics::PxRigidBody::SetKinematic(bool bKinematic)
{
	GetInternalObject().setRigidBodyFlag(physx::PxRigidBodyFlag::eKINEMATIC,bKinematic);
}
bool pragma::physics::PxRigidBody::IsKinematic() const
{
	return GetInternalObject().getRigidBodyFlags().isSet(physx::PxRigidBodyFlag::eKINEMATIC);
}

//////////////////

pragma::physics::PxSoftBody::PxSoftBody(IEnvironment &env,PxUniquePtr<physx::PxActor> actor,IShape &shape)
	: ISoftBody{env,shape,{}},PxCollisionObject{env,std::move(actor),shape}
{}
physx::PxActor &pragma::physics::PxSoftBody::GetInternalObject() const {return PxCollisionObject::GetInternalObject();}
#pragma optimize("",on)
