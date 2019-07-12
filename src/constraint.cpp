#include "pr_physx/constraint.hpp"
#include "pr_physx/environment.hpp"

pragma::physics::PhysXConstraint &pragma::physics::PhysXConstraint::GetConstraint(IConstraint &c)
{
	return *static_cast<PhysXConstraint*>(c.GetUserData());
}
const pragma::physics::PhysXConstraint &pragma::physics::PhysXConstraint::GetConstraint(const IConstraint &o) {return GetConstraint(const_cast<IConstraint&>(o));}
pragma::physics::PhysXConstraint::PhysXConstraint(IEnvironment &env,PhysXUniquePtr<physx::PxJoint> joint)
	: IConstraint{env},m_joint{std::move(joint)}
{
	SetUserData(this);
}
void pragma::physics::PhysXConstraint::Initialize()
{
	IConstraint::Initialize();
	GetInternalObject().userData = this;
}
void pragma::physics::PhysXConstraint::RemoveWorldObject() {}
void pragma::physics::PhysXConstraint::DoAddWorldObject() {}
physx::PxJoint &pragma::physics::PhysXConstraint::GetInternalObject() const {return *m_joint;}
pragma::physics::PhysXEnvironment &pragma::physics::PhysXConstraint::GetPxEnv() const {return static_cast<PhysXEnvironment&>(m_physEnv);}
void pragma::physics::PhysXConstraint::DoSetCollisionsEnabled(Bool b)
{
	// TODO
}
void pragma::physics::PhysXConstraint::SetEnabled(bool b)
{
	// TODO
}
bool pragma::physics::PhysXConstraint::IsEnabled() const
{
	// TODO
	return false;
}
void pragma::physics::PhysXConstraint::EnableCollisions()
{
	// TODO
}
void pragma::physics::PhysXConstraint::DisableCollisions()
{
	// TODO
}
pragma::physics::IRigidBody *pragma::physics::PhysXConstraint::GetSourceObject()
{
	// TODO
	return nullptr;
}
pragma::physics::IRigidBody *pragma::physics::PhysXConstraint::GetTargetObject()
{
	// TODO
	return nullptr;
}

void pragma::physics::PhysXConstraint::SetOverrideSolverIterationCount(int32_t count)
{
	// TODO
}
int32_t pragma::physics::PhysXConstraint::GetOverrideSolverIterationCount() const
{
	// TODO
	return 0;
}
float pragma::physics::PhysXConstraint::GetBreakingImpulseThreshold() const
{
	// TODO
	return 0.f;
}
void pragma::physics::PhysXConstraint::SetBreakingImpulseThreshold(float threshold)
{
	// TODO
}

/////////////

pragma::physics::PhysXFixedConstraint::PhysXFixedConstraint(IEnvironment &env,PhysXUniquePtr<physx::PxJoint> c)
	: IFixedConstraint{env},PhysXConstraint{env,std::move(c)},IConstraint{env}
{}

/////////////

pragma::physics::PhysXBallSocketConstraint::PhysXBallSocketConstraint(IEnvironment &env,PhysXUniquePtr<physx::PxJoint> c)
	: IBallSocketConstraint{env},PhysXConstraint{env,std::move(c)},IConstraint{env}
{}

/////////////

pragma::physics::PhysXHingeConstraint::PhysXHingeConstraint(IEnvironment &env,PhysXUniquePtr<physx::PxJoint> c)
	: IHingeConstraint{env},PhysXConstraint{env,std::move(c)},IConstraint{env}
{}

/////////////

pragma::physics::PhysXSliderConstraint::PhysXSliderConstraint(IEnvironment &env,PhysXUniquePtr<physx::PxJoint> c)
	: ISliderConstraint{env},PhysXConstraint{env,std::move(c)},IConstraint{env}
{}

/////////////

pragma::physics::PhysXConeTwistConstraint::PhysXConeTwistConstraint(IEnvironment &env,PhysXUniquePtr<physx::PxJoint> c)
	: IConeTwistConstraint{env},PhysXConstraint{env,std::move(c)},IConstraint{env}
{}
void pragma::physics::PhysXConeTwistConstraint::SetLimit(float swingSpan1,float swingSpan2,float twistSpan,float softness,float biasFactor,float relaxationFactor)
{
	// TODO
}

/////////////

pragma::physics::PhysXDoFConstraint::PhysXDoFConstraint(IEnvironment &env,PhysXUniquePtr<physx::PxJoint> c)
	: IDoFConstraint{env},PhysXConstraint{env,std::move(c)},IConstraint{env}
{}
void pragma::physics::PhysXDoFConstraint::SetLinearLimit(const Vector3 &lower,const Vector3 &upper)
{
	// TODO
}
void pragma::physics::PhysXDoFConstraint::SetLinearLimit(const Vector3 &lim)
{
	// TODO
}
void pragma::physics::PhysXDoFConstraint::SetLinearLowerLimit(const Vector3 &lim)
{
	// TODO
}
void pragma::physics::PhysXDoFConstraint::SetLinearUpperLimit(const Vector3 &lim)
{
	// TODO
}
void pragma::physics::PhysXDoFConstraint::SetAngularLimit(const EulerAngles &lower,const EulerAngles &upper)
{
	// TODO
}
void pragma::physics::PhysXDoFConstraint::SetAngularLimit(const EulerAngles &lim)
{
	// TODO
}
void pragma::physics::PhysXDoFConstraint::SetAngularLowerLimit(const EulerAngles &lim)
{
	// TODO
}
void pragma::physics::PhysXDoFConstraint::SetAngularUpperLimit(const EulerAngles &lim)
{
	// TODO
}

Vector3 pragma::physics::PhysXDoFConstraint::GetLinearLowerLimit() const
{
	// TODO
	return Vector3{};
}
Vector3 pragma::physics::PhysXDoFConstraint::GetlinearUpperLimit() const
{
	// TODO
	return Vector3{};
}
EulerAngles pragma::physics::PhysXDoFConstraint::GetAngularLowerLimit() const
{
	// TODO
	return EulerAngles{};
}
EulerAngles pragma::physics::PhysXDoFConstraint::GetAngularUpperLimit() const
{
	// TODO
	return EulerAngles{};
}

Vector3 pragma::physics::PhysXDoFConstraint::GetAngularTargetVelocity() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PhysXDoFConstraint::SetAngularTargetVelocity(const Vector3 &vel) const
{
	// TODO
}
Vector3 pragma::physics::PhysXDoFConstraint::GetAngularMaxMotorForce() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PhysXDoFConstraint::SetAngularMaxMotorForce(const Vector3 &force)
{
	// TODO
}
Vector3 pragma::physics::PhysXDoFConstraint::GetAngularMaxLimitForce() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PhysXDoFConstraint::SetAngularMaxLimitForce(const Vector3 &force)
{
	// TODO
}
Vector3 pragma::physics::PhysXDoFConstraint::GetAngularDamping() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PhysXDoFConstraint::SetAngularDamping(const Vector3 &damping)
{
	// TODO
}
Vector3 pragma::physics::PhysXDoFConstraint::GetAngularLimitSoftness() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PhysXDoFConstraint::SetAngularLimitSoftness(const Vector3 &softness)
{
	// TODO
}
Vector3 pragma::physics::PhysXDoFConstraint::GetAngularForceMixingFactor() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PhysXDoFConstraint::SetAngularForceMixingFactor(const Vector3 &factor)
{
	// TODO
}
Vector3 pragma::physics::PhysXDoFConstraint::GetAngularLimitErrorTolerance() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PhysXDoFConstraint::SetAngularLimitErrorTolerance(const Vector3 &tolerance)
{
	// TODO
}
Vector3 pragma::physics::PhysXDoFConstraint::GetAngularLimitForceMixingFactor() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PhysXDoFConstraint::SetAngularLimitForceMixingFactor(const Vector3 &factor)
{
	// TODO
}
Vector3 pragma::physics::PhysXDoFConstraint::GetAngularRestitutionFactor() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PhysXDoFConstraint::SetAngularRestitutionFactor(const Vector3 &factor)
{
	// TODO
}
bool pragma::physics::PhysXDoFConstraint::IsAngularMotorEnabled(uint8_t axis) const
{
	// TODO
	return false;
}
void pragma::physics::PhysXDoFConstraint::SetAngularMotorEnabled(uint8_t axis,bool bEnabled)
{
	// TODO
}
Vector3 pragma::physics::PhysXDoFConstraint::GetCurrentAngularLimitError() const
{
	// TODO
	return Vector3{};
}
Vector3 pragma::physics::PhysXDoFConstraint::GetCurrentAngularPosition() const
{
	// TODO
	return Vector3{};
}
Vector3i pragma::physics::PhysXDoFConstraint::GetCurrentAngularLimit() const
{
	// TODO
	return Vector3i{};
}
Vector3 pragma::physics::PhysXDoFConstraint::GetCurrentAngularAccumulatedImpulse() const
{
	// TODO
	return Vector3{};
}

Vector3 pragma::physics::PhysXDoFConstraint::GetLinearTargetVelocity() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PhysXDoFConstraint::SetLinearTargetVelocity(const Vector3 &vel) const
{
	// TODO
}
Vector3 pragma::physics::PhysXDoFConstraint::GetLinearMaxMotorForce() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PhysXDoFConstraint::SetLinearMaxMotorForce(const Vector3 &force)
{
	// TODO
}
float pragma::physics::PhysXDoFConstraint::GetLinearDamping() const
{
	// TODO
	return 0.f;
}
void pragma::physics::PhysXDoFConstraint::SetLinearDamping(float damping)
{
	// TODO
}
float pragma::physics::PhysXDoFConstraint::GetLinearLimitSoftness() const
{
	// TODO
	return 0.f;
}
void pragma::physics::PhysXDoFConstraint::SetLinearLimitSoftness(float softness)
{
	// TODO
}
Vector3 pragma::physics::PhysXDoFConstraint::GetLinearForceMixingFactor() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PhysXDoFConstraint::SetLinearForceMixingFactor(const Vector3 &factor)
{
	// TODO
}
Vector3 pragma::physics::PhysXDoFConstraint::GetLinearLimitErrorTolerance() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PhysXDoFConstraint::SetLinearLimitErrorTolerance(const Vector3 &tolerance)
{
	// TODO
}
Vector3 pragma::physics::PhysXDoFConstraint::GetLinearLimitForceMixingFactor() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PhysXDoFConstraint::SetLinearLimitForceMixingFactor(const Vector3 &factor)
{
	// TODO
}
float pragma::physics::PhysXDoFConstraint::GetLinearRestitutionFactor() const
{
	// TODO
	return 0.f;
}
void pragma::physics::PhysXDoFConstraint::SetLinearRestitutionFactor(float factor)
{
	// TODO
}
bool pragma::physics::PhysXDoFConstraint::IsLinearMotorEnabled(uint8_t axis) const
{
	// TODO
	return false;
}
void pragma::physics::PhysXDoFConstraint::SetLinearMotorEnabled(uint8_t axis,bool bEnabled)
{
	// TODO
}
Vector3 pragma::physics::PhysXDoFConstraint::GetCurrentLinearDifference() const
{
	// TODO
	return Vector3{};
}
Vector3 pragma::physics::PhysXDoFConstraint::GetCurrentLinearLimitError() const
{
	// TODO
	return Vector3{};
}
Vector3i pragma::physics::PhysXDoFConstraint::GetCurrentLinearLimit() const
{
	// TODO
	return Vector3i{};
}
Vector3 pragma::physics::PhysXDoFConstraint::GetCurrentLinearAccumulatedImpulse() const
{
	// TODO
	return Vector3{};
}

/////////////

pragma::physics::PhysXDoFSpringConstraint::PhysXDoFSpringConstraint(IEnvironment &env,PhysXUniquePtr<physx::PxJoint> c)
	: IDoFSpringConstraint{env},PhysXConstraint{env,std::move(c)},IConstraint{env}
{}
void pragma::physics::PhysXDoFSpringConstraint::CalculateTransforms()
{
	// TODO
}
void pragma::physics::PhysXDoFSpringConstraint::CalculateTransforms(const Transform &frameA,const Transform &frameB)
{
	// TODO
}
pragma::physics::Transform pragma::physics::PhysXDoFSpringConstraint::GetCalculatedTransformA() const
{
	// TODO
	return {};
}
pragma::physics::Transform pragma::physics::PhysXDoFSpringConstraint::GetCalculatedTransformB() const
{
	// TODO
	return {};
}
pragma::physics::Transform pragma::physics::PhysXDoFSpringConstraint::GetFrameOffsetA() const
{
	// TODO
	return {};
}
pragma::physics::Transform pragma::physics::PhysXDoFSpringConstraint::GetFrameOffsetB() const
{
	// TODO
	return {};
}
Vector3 pragma::physics::PhysXDoFSpringConstraint::GetAxis(pragma::Axis axisIndex) const
{
	// TODO
	return {};
}
double pragma::physics::PhysXDoFSpringConstraint::GetAngle(pragma::Axis axisIndex) const
{
	// TODO
	return {};
}
double pragma::physics::PhysXDoFSpringConstraint::GetRelativePivotPosition(pragma::Axis axisIndex) const
{
	// TODO
	return {};
}
void pragma::physics::PhysXDoFSpringConstraint::SetFrames(const Transform &frameA,const Transform &frameB)
{
	// TODO
}
void pragma::physics::PhysXDoFSpringConstraint::SetLinearLowerLimit(const Vector3 &linearLower)
{
	// TODO
}
Vector3 pragma::physics::PhysXDoFSpringConstraint::GetLinearLowerLimit() const
{
	// TODO
	return {};
}
void pragma::physics::PhysXDoFSpringConstraint::SetLinearUpperLimit(const Vector3 &linearUpper)
{
	// TODO
}
Vector3 pragma::physics::PhysXDoFSpringConstraint::GetLinearUpperLimit() const
{
	// TODO
	return {};
}
void pragma::physics::PhysXDoFSpringConstraint::SetAngularLowerLimit(const Vector3 &angularLower)
{
	// TODO
}
void pragma::physics::PhysXDoFSpringConstraint::SetAngularLowerLimitReversed(const Vector3 &angularLower)
{
	// TODO
}
Vector3 pragma::physics::PhysXDoFSpringConstraint::GetAngularLowerLimit() const
{
	// TODO
	return {};
}
Vector3 pragma::physics::PhysXDoFSpringConstraint::GetAngularLowerLimitReversed() const
{
	// TODO
	return {};
}
void pragma::physics::PhysXDoFSpringConstraint::SetAngularUpperLimit(const Vector3 &angularUpper)
{
	// TODO
}
void pragma::physics::PhysXDoFSpringConstraint::SetAngularUpperLimitReversed(const Vector3 &angularUpper)
{
	// TODO
}
Vector3 pragma::physics::PhysXDoFSpringConstraint::GetAngularUpperLimit() const
{
	// TODO
	return {};
}
Vector3 pragma::physics::PhysXDoFSpringConstraint::GetAngularUpperLimitReversed() const
{
	// TODO
	return {};
}
void pragma::physics::PhysXDoFSpringConstraint::SetLimit(AxisType type,pragma::Axis axis,double lo,double hi)
{
	// TODO
}
void pragma::physics::PhysXDoFSpringConstraint::SetLimitReversed(AxisType type,pragma::Axis axis,double lo,double hi)
{
	// TODO
}
bool pragma::physics::PhysXDoFSpringConstraint::IsLimited(AxisType type,pragma::Axis axis) const
{
	// TODO
	return {};
}
void pragma::physics::PhysXDoFSpringConstraint::SetRotationOrder(pragma::RotationOrder order)
{
	// TODO
}
pragma::RotationOrder pragma::physics::PhysXDoFSpringConstraint::GetRotationOrder() const
{
	// TODO
	return {};
}
void pragma::physics::PhysXDoFSpringConstraint::SetAxis(const Vector3 &axis1,const Vector3 &axis2)
{
	// TODO
}
void pragma::physics::PhysXDoFSpringConstraint::SetBounce(AxisType type,pragma::Axis axis,double bounce)
{
	// TODO
}
void pragma::physics::PhysXDoFSpringConstraint::EnableMotor(AxisType type,pragma::Axis axis,bool onOff)
{
	// TODO
}
void pragma::physics::PhysXDoFSpringConstraint::SetServo(AxisType type,pragma::Axis axis,bool onOff)
{
	// TODO
}
void pragma::physics::PhysXDoFSpringConstraint::SetTargetVelocity(AxisType type,pragma::Axis axis,double velocity)
{
	// TODO
}
void pragma::physics::PhysXDoFSpringConstraint::SetServoTarget(AxisType type,pragma::Axis axis,double target)
{
	// TODO
}
void pragma::physics::PhysXDoFSpringConstraint::SetMaxMotorForce(AxisType type,pragma::Axis axis,double force)
{
	// TODO
}
void pragma::physics::PhysXDoFSpringConstraint::EnableSpring(AxisType type,pragma::Axis axis,bool onOff)
{
	// TODO
}
void pragma::physics::PhysXDoFSpringConstraint::SetStiffness(AxisType type,pragma::Axis axis,double stiffness,bool limitIfNeeded)
{
	// TODO
}
void pragma::physics::PhysXDoFSpringConstraint::SetDamping(AxisType type,pragma::Axis axis,double damping,bool limitIfNeeded)
{
	// TODO
}
void pragma::physics::PhysXDoFSpringConstraint::SetEquilibriumPoint()
{
	// TODO
}
void pragma::physics::PhysXDoFSpringConstraint::SetEquilibriumPoint(AxisType type,pragma::Axis axis)
{
	// TODO
}
void pragma::physics::PhysXDoFSpringConstraint::SetEquilibriumPoint(AxisType type,pragma::Axis axis,double val)
{
	// TODO
}

void pragma::physics::PhysXDoFSpringConstraint::SetERP(AxisType type,pragma::Axis axis,double value)
{
	// TODO
}
double pragma::physics::PhysXDoFSpringConstraint::GetERP(AxisType type,pragma::Axis axis) const
{
	// TODO
	return {};
}
void pragma::physics::PhysXDoFSpringConstraint::SetStopERP(AxisType type,pragma::Axis axis,double value)
{
	// TODO
}
double pragma::physics::PhysXDoFSpringConstraint::GetStopERP(AxisType type,pragma::Axis axis) const
{
	// TODO
	return {};
}
void pragma::physics::PhysXDoFSpringConstraint::SetCFM(AxisType type,pragma::Axis axis,double value)
{
	// TODO
}
double pragma::physics::PhysXDoFSpringConstraint::GetCFM(AxisType type,pragma::Axis axis) const
{
	// TODO
	return {};
}
void pragma::physics::PhysXDoFSpringConstraint::SetStopCFM(AxisType type,pragma::Axis axis,double value)
{
	// TODO
}
double pragma::physics::PhysXDoFSpringConstraint::GetStopCFM(AxisType type,pragma::Axis axis) const
{
	// TODO
	return {};
}
