/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef __PR_PX_CONSTRAINT_HPP__
#define __PR_PX_CONSTRAINT_HPP__

#include <pragma/physics/constraint.hpp>
#include "common.hpp"

namespace physx
{
	class PxJoint;
	class PxFixedJoint;
	class PxSphericalJoint;
	class PxRevoluteJoint;
	class PxPrismaticJoint;
	class PxD6Joint;
};
namespace pragma::physics
{
	class PhysXEnvironment;
	class PhysXConstraint
		: virtual public IConstraint
	{
	public:
		enum class StateFlags : uint32_t
		{
			None = 0u,
			Enabled = 1u,
			Broken = Enabled<<1u
		};
		friend IEnvironment;
		static PhysXConstraint &GetConstraint(IConstraint &c);
		static const PhysXConstraint &GetConstraint(const IConstraint &c);
		physx::PxJoint &GetInternalObject() const;

		virtual void SetEnabled(bool b) override;
		virtual bool IsEnabled() const override;
		virtual bool IsBroken() const override;
		virtual void Break() override;
		virtual pragma::physics::IRigidBody *GetSourceActor() override;
		virtual pragma::physics::IRigidBody *GetTargetActor() override;

		virtual float GetBreakForce() const override;
		virtual void SetBreakForce(float force) override;
		virtual float GetBreakTorque() const override;
		virtual void SetBreakTorque(float torque) override;

		virtual void SetSoftness(float softness) override;
		virtual void SetDamping(float damping) override;
		virtual void SetRestitution(float restitution) override;

		virtual float GetSoftness() const override;
		virtual float GetDamping() const override;
		virtual float GetRestitution() const override;
	protected:
		PhysXConstraint(IEnvironment &env,PhysXUniquePtr<physx::PxJoint> joint);
		virtual void Initialize() override;
		virtual void RemoveWorldObject() override;
		virtual void DoAddWorldObject() override;
		PhysXEnvironment &GetPxEnv() const;
		virtual void DoSetCollisionsEnabled(Bool b) override;
		PhysXUniquePtr<physx::PxJoint> m_joint = px_null_ptr<physx::PxJoint>();
		StateFlags m_stateFlags = StateFlags::Enabled;
	};

	class PhysXFixedConstraint
		: public IFixedConstraint,
		public PhysXConstraint
	{
	public:
		friend IEnvironment;
	protected:
		PhysXFixedConstraint(IEnvironment &env,PhysXUniquePtr<physx::PxJoint> c);
	};

	class PhysXBallSocketConstraint
		: public IBallSocketConstraint,
		public PhysXConstraint
	{
	public:
		friend IEnvironment;
	protected:
		PhysXBallSocketConstraint(IEnvironment &env,PhysXUniquePtr<physx::PxJoint> c);
	};

	class PhysXHingeConstraint
		: public IHingeConstraint,
		public PhysXConstraint
	{
	public:
		friend IEnvironment;
		physx::PxRevoluteJoint &GetInternalObject() const;
		virtual void SetLimit(umath::Radian lowerLimit,umath::Radian upperLimit) override;
		virtual std::pair<umath::Radian,umath::Radian> GetLimit() const override;

		virtual void SetSoftness(float softness) override;
		virtual void SetDamping(float damping) override;
		virtual void SetRestitution(float restitution) override;
		virtual void DisableLimit() override;

		virtual float GetSoftness() const override;
		virtual float GetDamping() const override;
		virtual float GetRestitution() const override;
	protected:
		PhysXHingeConstraint(IEnvironment &env,PhysXUniquePtr<physx::PxJoint> c);
		void UpdateLimits();
		umath::Radian m_lowerLimit;
		umath::Radian m_upperLimit;
		float m_softness;
		float m_damping;
		float m_restitution;
	};

	class PhysXSliderConstraint
		: public ISliderConstraint,
		public PhysXConstraint
	{
	public:
		friend IEnvironment;
		physx::PxPrismaticJoint &GetInternalObject() const;
		virtual void SetLimit(float lowerLimit,float upperLimit) override;
		virtual void SetSoftness(float softness) override;
		virtual void SetDamping(float damping) override;
		virtual void SetRestitution(float restitution) override;
		virtual void DisableLimit() override;

		virtual std::pair<float,float> GetLimit() const override;
		virtual float GetSoftness() const override;
		virtual float GetDamping() const override;
		virtual float GetRestitution() const override;
	protected:
		PhysXSliderConstraint(IEnvironment &env,PhysXUniquePtr<physx::PxJoint> c);
		void UpdateLimits();
		float m_softness;
		float m_damping;
		float m_restitution;
		float m_lowerLimit;
		float m_upperLimit;
	};

	class PhysXConeTwistConstraint
		: public IConeTwistConstraint,
		public PhysXConstraint
	{
	public:
		friend IEnvironment;
		physx::PxD6Joint &GetInternalObject() const;
		virtual void SetLimit(float swingSpan1,float swingSpan2,float twistSpan) override;
		virtual void SetLimit(const Vector3 &lowerLimits,const Vector3 &upperLimits) override;
		virtual void GetLimit(float &outSwingSpan1,float &outSwingSpan2,float &outTwistSpan) override;

		virtual void SetSoftness(float softness) override;
		virtual void SetDamping(float damping) override;
		virtual void SetRestitution(float restitution) override;

		virtual float GetSoftness() const override;
		virtual float GetDamping() const override;
		virtual float GetRestitution() const override;
	protected:
		PhysXConeTwistConstraint(IEnvironment &env,PhysXUniquePtr<physx::PxJoint> c,const physx::PxTransform &pose0,const physx::PxTransform &pose1);
		void UpdateLimits();
		void UpdateLocalPoses();
		std::array<physx::PxTransform,2> m_localPoses;
		Vector3 m_lowerLimits;
		Vector3 m_upperLimits;
		float m_softness;
		float m_damping;
		float m_restitution;
	};

	class PhysXDoFConstraint
		: public IDoFConstraint,
		public PhysXConstraint
	{
	public:
		friend IEnvironment;
		virtual void SetLinearLimit(const Vector3 &lower,const Vector3 &upper) override;
		virtual void SetLinearLimit(const Vector3 &lim) override;
		virtual void SetLinearLowerLimit(const Vector3 &lim) override;
		virtual void SetLinearUpperLimit(const Vector3 &lim) override;
		virtual void SetAngularLimit(const EulerAngles &lower,const EulerAngles &upper) override;
		virtual void SetAngularLimit(const EulerAngles &lim) override;
		virtual void SetAngularLowerLimit(const EulerAngles &lim) override;
		virtual void SetAngularUpperLimit(const EulerAngles &lim) override;

		virtual Vector3 GetLinearLowerLimit() const override;
		virtual Vector3 GetlinearUpperLimit() const override;
		virtual EulerAngles GetAngularLowerLimit() const override;
		virtual EulerAngles GetAngularUpperLimit() const override;

		virtual Vector3 GetAngularTargetVelocity() const override;
		virtual void SetAngularTargetVelocity(const Vector3 &vel) const override;
		virtual Vector3 GetAngularMaxMotorForce() const override;
		virtual void SetAngularMaxMotorForce(const Vector3 &force) override;
		virtual Vector3 GetAngularMaxLimitForce() const override;
		virtual void SetAngularMaxLimitForce(const Vector3 &force) override;
		virtual Vector3 GetAngularDamping() const override;
		virtual void SetAngularDamping(const Vector3 &damping) override;
		virtual Vector3 GetAngularLimitSoftness() const override;
		virtual void SetAngularLimitSoftness(const Vector3 &softness) override;
		virtual Vector3 GetAngularForceMixingFactor() const override;
		virtual void SetAngularForceMixingFactor(const Vector3 &factor) override;
		virtual Vector3 GetAngularLimitErrorTolerance() const override;
		virtual void SetAngularLimitErrorTolerance(const Vector3 &tolerance) override;
		virtual Vector3 GetAngularLimitForceMixingFactor() const override;
		virtual void SetAngularLimitForceMixingFactor(const Vector3 &factor) override;
		virtual Vector3 GetAngularRestitutionFactor() const override;
		virtual void SetAngularRestitutionFactor(const Vector3 &factor) override;
		virtual bool IsAngularMotorEnabled(uint8_t axis) const override;
		virtual void SetAngularMotorEnabled(uint8_t axis,bool bEnabled) override;
		virtual Vector3 GetCurrentAngularLimitError() const override;
		virtual Vector3 GetCurrentAngularPosition() const override;
		virtual Vector3i GetCurrentAngularLimit() const override;
		virtual Vector3 GetCurrentAngularAccumulatedImpulse() const override;

		virtual Vector3 GetLinearTargetVelocity() const override;
		virtual void SetLinearTargetVelocity(const Vector3 &vel) const override;
		virtual Vector3 GetLinearMaxMotorForce() const override;
		virtual void SetLinearMaxMotorForce(const Vector3 &force) override;
		virtual float GetLinearDamping() const override;
		virtual void SetLinearDamping(float damping) override;
		virtual float GetLinearLimitSoftness() const override;
		virtual void SetLinearLimitSoftness(float softness) override;
		virtual Vector3 GetLinearForceMixingFactor() const override;
		virtual void SetLinearForceMixingFactor(const Vector3 &factor) override;
		virtual Vector3 GetLinearLimitErrorTolerance() const override;
		virtual void SetLinearLimitErrorTolerance(const Vector3 &tolerance) override;
		virtual Vector3 GetLinearLimitForceMixingFactor() const override;
		virtual void SetLinearLimitForceMixingFactor(const Vector3 &factor) override;
		virtual float GetLinearRestitutionFactor() const override;
		virtual void SetLinearRestitutionFactor(float factor) override;
		virtual bool IsLinearMotorEnabled(uint8_t axis) const override;
		virtual void SetLinearMotorEnabled(uint8_t axis,bool bEnabled) override;
		virtual Vector3 GetCurrentLinearDifference() const override;
		virtual Vector3 GetCurrentLinearLimitError() const override;
		virtual Vector3i GetCurrentLinearLimit() const override;
		virtual Vector3 GetCurrentLinearAccumulatedImpulse() const override;
	protected:
		PhysXDoFConstraint(IEnvironment &env,PhysXUniquePtr<physx::PxJoint> c);
	};

	class PhysXDoFSpringConstraint
		: public IDoFSpringConstraint,
		public PhysXConstraint
	{
	public:
		friend IEnvironment;

		virtual void CalculateTransforms() override;
		virtual void CalculateTransforms(const umath::Transform &frameA,const umath::Transform &frameB) override;
		virtual umath::Transform GetCalculatedTransformA() const override;
		virtual umath::Transform GetCalculatedTransformB() const override;
		virtual umath::Transform GetFrameOffsetA() const override;
		virtual umath::Transform GetFrameOffsetB() const override;
		virtual Vector3 GetAxis(pragma::Axis axisIndex) const override;
		virtual double GetAngle(pragma::Axis axisIndex) const override;
		virtual double GetRelativePivotPosition(pragma::Axis axisIndex) const override;
		virtual void SetFrames(const umath::Transform &frameA,const umath::Transform &frameB) override;
		virtual void SetLinearLowerLimit(const Vector3 &linearLower) override;
		virtual Vector3 GetLinearLowerLimit() const override;
		virtual void SetLinearUpperLimit(const Vector3 &linearUpper) override;
		virtual Vector3 GetLinearUpperLimit() const override;
		virtual void SetAngularLowerLimit(const Vector3 &angularLower) override;
		virtual void SetAngularLowerLimitReversed(const Vector3 &angularLower) override;
		virtual Vector3 GetAngularLowerLimit() const override;
		virtual Vector3 GetAngularLowerLimitReversed() const override;
		virtual void SetAngularUpperLimit(const Vector3 &angularUpper) override;
		virtual void SetAngularUpperLimitReversed(const Vector3 &angularUpper) override;
		virtual Vector3 GetAngularUpperLimit() const override;
		virtual Vector3 GetAngularUpperLimitReversed() const override;
		virtual void SetLimit(AxisType type,pragma::Axis axis,double lo,double hi) override;
		virtual void SetLimitReversed(AxisType type,pragma::Axis axis,double lo,double hi) override;
		virtual bool IsLimited(AxisType type,pragma::Axis axis) const override;
		virtual void SetRotationOrder(pragma::RotationOrder order) override;
		virtual pragma::RotationOrder GetRotationOrder() const override;
		virtual void SetAxis(const Vector3 &axis1,const Vector3 &axis2) override;
		virtual void SetBounce(AxisType type,pragma::Axis axis,double bounce) override;
		virtual void EnableMotor(AxisType type,pragma::Axis axis,bool onOff) override;
		virtual void SetServo(AxisType type,pragma::Axis axis,bool onOff) override;
		virtual void SetTargetVelocity(AxisType type,pragma::Axis axis,double velocity) override;
		virtual void SetServoTarget(AxisType type,pragma::Axis axis,double target) override;
		virtual void SetMaxMotorForce(AxisType type,pragma::Axis axis,double force) override;
		virtual void EnableSpring(AxisType type,pragma::Axis axis,bool onOff) override;
		virtual void SetStiffness(AxisType type,pragma::Axis axis,double stiffness,bool limitIfNeeded=true) override;
		virtual void SetDamping(AxisType type,pragma::Axis axis,double damping,bool limitIfNeeded=true) override;
		virtual void SetEquilibriumPoint() override;
		virtual void SetEquilibriumPoint(AxisType type,pragma::Axis axis) override;
		virtual void SetEquilibriumPoint(AxisType type,pragma::Axis axis,double val) override;

		virtual void SetERP(AxisType type,pragma::Axis axis,double value) override;
		virtual double GetERP(AxisType type,pragma::Axis axis) const override;
		virtual void SetStopERP(AxisType type,pragma::Axis axis,double value) override;
		virtual double GetStopERP(AxisType type,pragma::Axis axis) const override;
		virtual void SetCFM(AxisType type,pragma::Axis axis,double value) override;
		virtual double GetCFM(AxisType type,pragma::Axis axis) const override;
		virtual void SetStopCFM(AxisType type,pragma::Axis axis,double value) override;
		virtual double GetStopCFM(AxisType type,pragma::Axis axis) const override;
	protected:
		PhysXDoFSpringConstraint(IEnvironment &env,PhysXUniquePtr<physx::PxJoint> c);
	};
};
REGISTER_BASIC_BITWISE_OPERATORS(pragma::physics::PhysXConstraint::StateFlags)

#endif
