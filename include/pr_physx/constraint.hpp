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
	class PxEnvironment;
	class PxConstraint
		: virtual public IConstraint
	{
	public:
		friend IEnvironment;
		static PxConstraint &GetConstraint(IConstraint &c);
		static const PxConstraint &GetConstraint(const IConstraint &c);
		physx::PxJoint &GetInternalObject() const;

		virtual void Initialize() override;
		virtual void SetEnabled(bool b) override;
		virtual bool IsEnabled() const override;
		virtual void EnableCollisions() override;
		virtual void DisableCollisions() override;
		virtual pragma::physics::IRigidBody *GetSourceObject() override;
		virtual pragma::physics::IRigidBody *GetTargetObject() override;

		virtual void SetOverrideSolverIterationCount(int32_t count) override;
		virtual int32_t GetOverrideSolverIterationCount() const override;
		virtual float GetBreakingImpulseThreshold() const override;
		virtual void SetBreakingImpulseThreshold(float threshold) override;
	protected:
		PxConstraint(IEnvironment &env,PxUniquePtr<physx::PxJoint> joint);
		PxEnvironment &GetPxEnv() const;
		virtual void DoSetCollisionsEnabled(Bool b) override;
		PxUniquePtr<physx::PxJoint> m_joint = px_null_ptr<physx::PxJoint>();
	};

	class PxFixedConstraint
		: public IFixedConstraint,
		public PxConstraint
	{
	public:
		friend IEnvironment;
	protected:
		PxFixedConstraint(IEnvironment &env,PxUniquePtr<physx::PxFixedJoint> c);
	};

	class PxBallSocketConstraint
		: public IBallSocketConstraint,
		public PxConstraint
	{
	public:
		friend IEnvironment;
	protected:
		PxBallSocketConstraint(IEnvironment &env,PxUniquePtr<physx::PxSphericalJoint> c);
	};

	class PxHingeConstraint
		: public IHingeConstraint,
		public PxConstraint
	{
	public:
		friend IEnvironment;
	protected:
		PxHingeConstraint(IEnvironment &env,PxUniquePtr<physx::PxRevoluteJoint> c);
	};

	class PxSliderConstraint
		: public ISliderConstraint,
		public PxConstraint
	{
	public:
		friend IEnvironment;
	protected:
		PxSliderConstraint(IEnvironment &env,PxUniquePtr<physx::PxPrismaticJoint> c);
	};

	class PxConeTwistConstraint
		: public IConeTwistConstraint,
		public PxConstraint
	{
	public:
		friend IEnvironment;
		virtual void SetLimit(float swingSpan1,float swingSpan2,float twistSpan,float softness=1.f,float biasFactor=0.3f,float relaxationFactor=1.f) override;
	protected:
		PxConeTwistConstraint(IEnvironment &env,PxUniquePtr<physx::PxSphericalJoint> c);
	};

	class PxDoFConstraint
		: public IDoFConstraint,
		public PxConstraint
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
		PxDoFConstraint(IEnvironment &env,PxUniquePtr<physx::PxD6Joint> c);
	};

	class PxDoFSpringConstraint
		: public IDoFSpringConstraint,
		public PxConstraint
	{
	public:
		friend IEnvironment;

		virtual void CalculateTransforms() override;
		virtual void CalculateTransforms(const Transform &frameA,const Transform &frameB) override;
		virtual Transform GetCalculatedTransformA() const override;
		virtual Transform GetCalculatedTransformB() const override;
		virtual Transform GetFrameOffsetA() const override;
		virtual Transform GetFrameOffsetB() const override;
		virtual Vector3 GetAxis(pragma::Axis axisIndex) const override;
		virtual double GetAngle(pragma::Axis axisIndex) const override;
		virtual double GetRelativePivotPosition(pragma::Axis axisIndex) const override;
		virtual void SetFrames(const Transform &frameA,const Transform &frameB) override;
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
		PxDoFSpringConstraint(IEnvironment &env,PxUniquePtr<physx::PxD6Joint> c);
	};
};

#endif
