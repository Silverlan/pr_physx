#ifndef __PR_PX_COLLISION_OBJECT_HPP__
#define __PR_PX_COLLISION_OBJECT_HPP__

#include <pragma/physics/collision_object.hpp>
#include "pr_physx/common.hpp"

namespace physx
{
	class PxActor;
	class PxRigidDynamic;
};
namespace pragma::physics
{
	class PxEnvironment;
	class PxCollisionObject
		: virtual public pragma::physics::ICollisionObject
	{
	public:
		friend IEnvironment;
		static PxCollisionObject &GetCollisionObject(ICollisionObject &o);
		static const PxCollisionObject &GetCollisionObject(const ICollisionObject &o);
		PxCollisionObject(IEnvironment &env,PxUniquePtr<physx::PxActor> actor,IShape &shape);
		physx::PxActor &GetInternalObject() const;
		virtual void GetAABB(Vector3 &min,Vector3 &max) const override;
	protected:
		PxEnvironment &GetPxEnv() const;
	private:
		virtual void RemoveWorldObject() override;
		virtual void DoAddWorldObject() override;
		PxUniquePtr<physx::PxActor> m_actor = px_null_ptr<physx::PxActor>();
	};
	class PxRigidBody
		: virtual public pragma::physics::IRigidBody,
		public PxCollisionObject
	{
	public:
		friend IEnvironment;
		PxRigidBody(IEnvironment &env,PxUniquePtr<physx::PxRigidDynamic> actor,IShape &shape,float mass,const Vector3 &localInertia);
		physx::PxRigidDynamic &GetInternalObject() const;

		// Collision object
		virtual void SetActivationState(ActivationState state) override;
		virtual ActivationState GetActivationState() const override;

		virtual void SetContactProcessingThreshold(float threshold) override;
		virtual bool IsStatic() const override;
		virtual void SetStatic(bool b) override;

		virtual void SetCCDEnabled(bool b) override;
		virtual Vector3 GetPos() const override;
		virtual void SetPos(const Vector3 &pos) override;
		virtual Quat GetRotation() const override;
		virtual void SetRotation(const Quat &rot) override;
		virtual Transform GetWorldTransform() override;
		virtual void SetWorldTransform(const Transform &t) override;
		virtual void WakeUp(bool forceActivation=false) override;
		virtual void PutToSleep() override;
		virtual bool IsAsleep() const override;

		virtual bool IsTrigger() override;

		virtual void SetSimulationEnabled(bool b) override;
		virtual bool IsSimulationEnabled() const override;
		virtual void SetCollisionsEnabled(bool enabled) override;
		//

		virtual void ApplyForce(const Vector3 &force) override;
		virtual void ApplyForce(const Vector3 &force,const Vector3 &relPos) override;
		virtual void ApplyImpulse(const Vector3 &impulse) override;
		virtual void ApplyImpulse(const Vector3 &impulse,const Vector3 &relPos) override;
		virtual void ApplyTorque(const Vector3 &torque) override;
		virtual void ApplyTorqueImpulse(const Vector3 &torque) override;
		virtual void ClearForces() override;
		virtual Vector3 GetTotalForce() override;
		virtual Vector3 GetTotalTorque() override;
		virtual void SetMassProps(float mass,const Vector3 &inertia) override;
		virtual float GetMass() const override;
		virtual void SetMass(float mass) override;
		virtual Vector3 &GetInertia() override;
		virtual Mat3 GetInvInertiaTensorWorld() const override;
		virtual void SetInertia(const Vector3 &inertia) override;
		virtual Vector3 GetLinearVelocity() const override;
		virtual Vector3 GetAngularVelocity() const override;
		virtual void SetLinearVelocity(const Vector3 &vel) override;
		virtual void SetAngularVelocity(const Vector3 &vel) override;
		virtual void SetLinearFactor(const Vector3 &factor) override;
		virtual void SetAngularFactor(const Vector3 &factor) override;
		virtual Vector3 GetLinearFactor() const override;
		virtual Vector3 GetAngularFactor() const override;
		virtual void SetLinearDamping(float damping) override;
		virtual void SetAngularDamping(float damping) override;
		virtual float GetLinearDamping() const override;
		virtual float GetAngularDamping() const override;
		virtual void SetLinearSleepingThreshold(float threshold) override;
		virtual void SetAngularSleepingThreshold(float threshold) override;
		virtual float GetLinearSleepingThreshold() const override;
		virtual float GetAngularSleepingThreshold() const override;

		virtual void SetKinematic(bool bKinematic) override;
		virtual bool IsKinematic() const override;
	private:
		// Collision object
		virtual void ApplyCollisionShape(pragma::physics::IShape *optShape) override;
		virtual void DoSetCollisionFilterGroup(CollisionMask group) override;
		virtual void DoSetCollisionFilterMask(CollisionMask mask) override;
		//
	};
	class PxSoftBody
		: virtual public pragma::physics::ISoftBody,
		public PxCollisionObject
	{
	public:
		friend IEnvironment;
		PxSoftBody(IEnvironment &env,PxUniquePtr<physx::PxActor> actor,IShape &shape);
		physx::PxActor &GetInternalObject() const;
	};
};

#endif
