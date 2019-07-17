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
	class PhysXController;
	class PhysXEnvironment;
	class PhysXActorShape;

	struct NoCollisionCategory;
	using NoCollisionCategoryId = uint32_t;
	class PhysXMaterial;
	class PhysXShape;
	class PhysXCollisionObject;
	class PhysXRigidBody;
	class PhysXActorShapeCollection
	{
	public:
		friend PhysXCollisionObject;
		friend PhysXRigidBody;
		PhysXActorShapeCollection(PhysXCollisionObject &colObj);
		const std::vector<std::unique_ptr<PhysXActorShape>> &GetActorShapes() const;

		PhysXActorShape *AttachShapeToActor(PhysXShape &shape,PhysXMaterial &mat);
		// Adds the shape to the shape list, but does not attach it to the
		// actor. Assumes that it already has been attached to it!
		PhysXActorShape *AddShape(PhysXShape &shape,physx::PxShape &actorShape);

		void SetTrigger(bool bTrigger);
		bool IsTrigger() const;

		void ApplySurfaceMaterial(PhysXMaterial &mat);

		void SetLocalPose(const Transform &t);
		Transform GetLocalPose() const;
		void CalcMassProps(float mass,Vector3 &centerOfMass);
	private:
		void Clear();
		PhysXActorShape *AddShape(PhysXShape &shape,physx::PxShape &actorShape,bool applyPose);

		// Contains one shape if created from a non-compound geometry,
		// otherwise can contain more than one shape.
		std::vector<std::unique_ptr<PhysXActorShape>> m_actorShapes = {};
		PhysXCollisionObject &m_collisionObject;
	};

	class PhysXCollisionObject
		: virtual public pragma::physics::ICollisionObject
	{
	public:
		friend IEnvironment;
		static PhysXCollisionObject &GetCollisionObject(ICollisionObject &o);
		static const PhysXCollisionObject &GetCollisionObject(const ICollisionObject &o);
		PhysXCollisionObject(IEnvironment &env,PhysXUniquePtr<physx::PxActor> actor,IShape &shape);
		PhysXEnvironment &GetPxEnv() const;
		physx::PxActor &GetInternalObject() const;
		NoCollisionCategoryId DisableSelfCollisions();
		virtual void GetAABB(Vector3 &min,Vector3 &max) const override;
		virtual void SetSleepReportEnabled(bool reportEnabled) override;
		virtual bool IsSleepReportEnabled() const override;

		virtual void SetTrigger(bool bTrigger) override;
		virtual bool IsTrigger() const override;

		virtual void SetLocalPose(const Transform &t) override;
		virtual Transform GetLocalPose() const override;

		PhysXActorShapeCollection &GetActorShapeCollection() const;
	protected:
		virtual void Initialize() override;
		virtual void OnRemove() override;
		virtual void RemoveWorldObject() override;
		virtual void DoAddWorldObject() override;
		mutable PhysXActorShapeCollection m_actorShapeCollection;
		PhysXUniquePtr<NoCollisionCategory> m_noCollisionCategory = px_null_ptr<NoCollisionCategory>();
	private:
		PhysXUniquePtr<physx::PxActor> m_actor = px_null_ptr<physx::PxActor>();
	};
	class PhysXRigidBody
		: virtual public pragma::physics::IRigidBody,
		public PhysXCollisionObject
	{
	public:
		friend IEnvironment;
		physx::PxRigidActor &GetInternalObject() const;

		// Collision object
		virtual void SetContactProcessingThreshold(float threshold) override;

		virtual Vector3 GetPos() const override;
		virtual void SetPos(const Vector3 &pos) override;
		virtual Quat GetRotation() const override;
		virtual void SetRotation(const Quat &rot) override;
		virtual Transform GetWorldTransform() override;
		virtual void SetWorldTransform(const Transform &t) override;

		virtual void SetSimulationEnabled(bool b) override;
		virtual bool IsSimulationEnabled() const override;
		virtual void SetCollisionsEnabled(bool enabled) override;
		//

		virtual void SetCenterOfMassOffset(const Vector3 &offset) override;
		virtual Vector3 GetCenterOfMassOffset() const override;

		virtual void SetMassProps(float mass,const Vector3 &inertia) override;
		virtual Vector3 &GetInertia() override;
		virtual Mat3 GetInvInertiaTensorWorld() const override;
		virtual void SetInertia(const Vector3 &inertia) override;

		void SetController(PhysXController &controller);
		PhysXController *GetController() const;
	protected:
		PhysXRigidBody(IEnvironment &env,PhysXUniquePtr<physx::PxActor> actor,IShape &shape);
		virtual void ApplyCollisionShape(pragma::physics::IShape *optShape) override;
		virtual void RemoveWorldObject() override;
		virtual void DoAddWorldObject() override;
	private:
		// Collision object
		virtual void DoSetCollisionFilterGroup(CollisionMask group) override;
		virtual void DoSetCollisionFilterMask(CollisionMask mask) override;
		//

		// The controller this body belongs to
		mutable util::TWeakSharedHandle<PhysXController> m_controller = {};
	};
	class PhysXRigidDynamic
		: public PhysXRigidBody
	{
	public:
		friend IEnvironment;
		physx::PxRigidDynamic &GetInternalObject() const;
		virtual void SetActivationState(ActivationState state) override;
		virtual ActivationState GetActivationState() const override;
		virtual bool IsStatic() const override;
		virtual void SetStatic(bool b) override;
		virtual void WakeUp(bool forceActivation=false) override;
		virtual void PutToSleep() override;
		virtual bool IsAsleep() const override;

		virtual void SetCCDEnabled(bool b) override;
		virtual void ApplyForce(const Vector3 &force) override;
		virtual void ApplyForce(const Vector3 &force,const Vector3 &relPos) override;
		virtual void ApplyImpulse(const Vector3 &impulse) override;
		virtual void ApplyImpulse(const Vector3 &impulse,const Vector3 &relPos) override;
		virtual void ApplyTorque(const Vector3 &torque) override;
		virtual void ApplyTorqueImpulse(const Vector3 &torque) override;
		virtual void ClearForces() override;
		virtual Vector3 GetTotalForce() const override;
		virtual Vector3 GetTotalTorque() const override;
		virtual float GetMass() const override;
		virtual void SetMass(float mass) override;
		virtual Vector3 GetCenterOfMass() const override;
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

		virtual void SetCenterOfMassOffset(const Vector3 &offset) override;
		virtual Vector3 GetCenterOfMassOffset() const override;

		virtual void SetKinematic(bool bKinematic) override;
		virtual bool IsKinematic() const override;
	protected:
		PhysXRigidDynamic(IEnvironment &env,PhysXUniquePtr<physx::PxActor> actor,IShape &shape);
	private:
		virtual void ApplyCollisionShape(pragma::physics::IShape *optShape) override;
	};
	class PhysXRigidStatic
		: public PhysXRigidBody
	{
	public:
		friend IEnvironment;
		physx::PxRigidStatic &GetInternalObject() const;
		virtual void SetActivationState(ActivationState state) override;
		virtual ActivationState GetActivationState() const override;
		virtual bool IsStatic() const override;
		virtual void SetStatic(bool b) override;
		virtual void WakeUp(bool forceActivation=false) override;
		virtual void PutToSleep() override;
		virtual bool IsAsleep() const override;

		virtual void SetCCDEnabled(bool b) override;
		virtual void ApplyForce(const Vector3 &force) override;
		virtual void ApplyForce(const Vector3 &force,const Vector3 &relPos) override;
		virtual void ApplyImpulse(const Vector3 &impulse) override;
		virtual void ApplyImpulse(const Vector3 &impulse,const Vector3 &relPos) override;
		virtual void ApplyTorque(const Vector3 &torque) override;
		virtual void ApplyTorqueImpulse(const Vector3 &torque) override;
		virtual void ClearForces() override;
		virtual Vector3 GetTotalForce() const override;
		virtual Vector3 GetTotalTorque() const override;
		virtual float GetMass() const override;
		virtual void SetMass(float mass) override;
		virtual Vector3 GetCenterOfMass() const override;
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
	protected:
		PhysXRigidStatic(IEnvironment &env,PhysXUniquePtr<physx::PxActor> actor,IShape &shape);
	};
	class PhysXSoftBody
		: virtual public pragma::physics::ISoftBody,
		public PhysXCollisionObject
	{
	public:
		friend IEnvironment;
		PhysXSoftBody(IEnvironment &env,PhysXUniquePtr<physx::PxActor> actor,IShape &shape);
		physx::PxActor &GetInternalObject() const;
	};
};

#endif
