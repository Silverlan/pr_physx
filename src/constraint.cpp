#include "pr_physx/constraint.hpp"
#include "pr_physx/environment.hpp"

pragma::physics::PxConstraint &pragma::physics::PxConstraint::GetConstraint(IConstraint &c)
{
	return *static_cast<PxConstraint*>(c.GetUserData());
}
const pragma::physics::PxConstraint &pragma::physics::PxConstraint::GetConstraint(const IConstraint &o) {return GetConstraint(const_cast<IConstraint&>(o));}
pragma::physics::PxConstraint::PxConstraint(IEnvironment &env,PxUniquePtr<physx::PxJoint> joint)
	: IConstraint{env},m_joint{std::move(joint)}
{
	SetUserData(this);
}
void pragma::physics::PxConstraint::Initialize()
{
	IConstraint::Initialize();
	GetInternalObject().userData = this;
}
void pragma::physics::PxConstraint::RemoveWorldObject() {}
void pragma::physics::PxConstraint::DoAddWorldObject() {}
physx::PxJoint &pragma::physics::PxConstraint::GetInternalObject() const {return *m_joint;}
pragma::physics::PxEnvironment &pragma::physics::PxConstraint::GetPxEnv() const {return static_cast<PxEnvironment&>(m_physEnv);}
void pragma::physics::PxConstraint::DoSetCollisionsEnabled(Bool b)
{
	// TODO
}
void pragma::physics::PxConstraint::SetEnabled(bool b)
{
	// TODO
}
bool pragma::physics::PxConstraint::IsEnabled() const
{
	// TODO
	return false;
}
void pragma::physics::PxConstraint::EnableCollisions()
{
	// TODO
}
void pragma::physics::PxConstraint::DisableCollisions()
{
	// TODO
}
pragma::physics::IRigidBody *pragma::physics::PxConstraint::GetSourceObject()
{
	// TODO
	return nullptr;
}
pragma::physics::IRigidBody *pragma::physics::PxConstraint::GetTargetObject()
{
	// TODO
	return nullptr;
}

void pragma::physics::PxConstraint::SetOverrideSolverIterationCount(int32_t count)
{
	// TODO
}
int32_t pragma::physics::PxConstraint::GetOverrideSolverIterationCount() const
{
	// TODO
	return 0;
}
float pragma::physics::PxConstraint::GetBreakingImpulseThreshold() const
{
	// TODO
	return 0.f;
}
void pragma::physics::PxConstraint::SetBreakingImpulseThreshold(float threshold)
{
	// TODO
}

/////////////

pragma::physics::PxFixedConstraint::PxFixedConstraint(IEnvironment &env,PxUniquePtr<physx::PxJoint> c)
	: IFixedConstraint{env},PxConstraint{env,std::move(c)},IConstraint{env}
{}

/////////////

pragma::physics::PxBallSocketConstraint::PxBallSocketConstraint(IEnvironment &env,PxUniquePtr<physx::PxJoint> c)
	: IBallSocketConstraint{env},PxConstraint{env,std::move(c)},IConstraint{env}
{}

/////////////

pragma::physics::PxHingeConstraint::PxHingeConstraint(IEnvironment &env,PxUniquePtr<physx::PxJoint> c)
	: IHingeConstraint{env},PxConstraint{env,std::move(c)},IConstraint{env}
{}

/////////////

pragma::physics::PxSliderConstraint::PxSliderConstraint(IEnvironment &env,PxUniquePtr<physx::PxJoint> c)
	: ISliderConstraint{env},PxConstraint{env,std::move(c)},IConstraint{env}
{}

/////////////

pragma::physics::PxConeTwistConstraint::PxConeTwistConstraint(IEnvironment &env,PxUniquePtr<physx::PxJoint> c)
	: IConeTwistConstraint{env},PxConstraint{env,std::move(c)},IConstraint{env}
{}
void pragma::physics::PxConeTwistConstraint::SetLimit(float swingSpan1,float swingSpan2,float twistSpan,float softness,float biasFactor,float relaxationFactor)
{
	// TODO
}

/////////////

pragma::physics::PxDoFConstraint::PxDoFConstraint(IEnvironment &env,PxUniquePtr<physx::PxJoint> c)
	: IDoFConstraint{env},PxConstraint{env,std::move(c)},IConstraint{env}
{}
void pragma::physics::PxDoFConstraint::SetLinearLimit(const Vector3 &lower,const Vector3 &upper)
{
	// TODO
}
void pragma::physics::PxDoFConstraint::SetLinearLimit(const Vector3 &lim)
{
	// TODO
}
void pragma::physics::PxDoFConstraint::SetLinearLowerLimit(const Vector3 &lim)
{
	// TODO
}
void pragma::physics::PxDoFConstraint::SetLinearUpperLimit(const Vector3 &lim)
{
	// TODO
}
void pragma::physics::PxDoFConstraint::SetAngularLimit(const EulerAngles &lower,const EulerAngles &upper)
{
	// TODO
}
void pragma::physics::PxDoFConstraint::SetAngularLimit(const EulerAngles &lim)
{
	// TODO
}
void pragma::physics::PxDoFConstraint::SetAngularLowerLimit(const EulerAngles &lim)
{
	// TODO
}
void pragma::physics::PxDoFConstraint::SetAngularUpperLimit(const EulerAngles &lim)
{
	// TODO
}

Vector3 pragma::physics::PxDoFConstraint::GetLinearLowerLimit() const
{
	// TODO
	return Vector3{};
}
Vector3 pragma::physics::PxDoFConstraint::GetlinearUpperLimit() const
{
	// TODO
	return Vector3{};
}
EulerAngles pragma::physics::PxDoFConstraint::GetAngularLowerLimit() const
{
	// TODO
	return EulerAngles{};
}
EulerAngles pragma::physics::PxDoFConstraint::GetAngularUpperLimit() const
{
	// TODO
	return EulerAngles{};
}

Vector3 pragma::physics::PxDoFConstraint::GetAngularTargetVelocity() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PxDoFConstraint::SetAngularTargetVelocity(const Vector3 &vel) const
{
	// TODO
}
Vector3 pragma::physics::PxDoFConstraint::GetAngularMaxMotorForce() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PxDoFConstraint::SetAngularMaxMotorForce(const Vector3 &force)
{
	// TODO
}
Vector3 pragma::physics::PxDoFConstraint::GetAngularMaxLimitForce() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PxDoFConstraint::SetAngularMaxLimitForce(const Vector3 &force)
{
	// TODO
}
Vector3 pragma::physics::PxDoFConstraint::GetAngularDamping() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PxDoFConstraint::SetAngularDamping(const Vector3 &damping)
{
	// TODO
}
Vector3 pragma::physics::PxDoFConstraint::GetAngularLimitSoftness() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PxDoFConstraint::SetAngularLimitSoftness(const Vector3 &softness)
{
	// TODO
}
Vector3 pragma::physics::PxDoFConstraint::GetAngularForceMixingFactor() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PxDoFConstraint::SetAngularForceMixingFactor(const Vector3 &factor)
{
	// TODO
}
Vector3 pragma::physics::PxDoFConstraint::GetAngularLimitErrorTolerance() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PxDoFConstraint::SetAngularLimitErrorTolerance(const Vector3 &tolerance)
{
	// TODO
}
Vector3 pragma::physics::PxDoFConstraint::GetAngularLimitForceMixingFactor() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PxDoFConstraint::SetAngularLimitForceMixingFactor(const Vector3 &factor)
{
	// TODO
}
Vector3 pragma::physics::PxDoFConstraint::GetAngularRestitutionFactor() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PxDoFConstraint::SetAngularRestitutionFactor(const Vector3 &factor)
{
	// TODO
}
bool pragma::physics::PxDoFConstraint::IsAngularMotorEnabled(uint8_t axis) const
{
	// TODO
	return false;
}
void pragma::physics::PxDoFConstraint::SetAngularMotorEnabled(uint8_t axis,bool bEnabled)
{
	// TODO
}
Vector3 pragma::physics::PxDoFConstraint::GetCurrentAngularLimitError() const
{
	// TODO
	return Vector3{};
}
Vector3 pragma::physics::PxDoFConstraint::GetCurrentAngularPosition() const
{
	// TODO
	return Vector3{};
}
Vector3i pragma::physics::PxDoFConstraint::GetCurrentAngularLimit() const
{
	// TODO
	return Vector3i{};
}
Vector3 pragma::physics::PxDoFConstraint::GetCurrentAngularAccumulatedImpulse() const
{
	// TODO
	return Vector3{};
}

Vector3 pragma::physics::PxDoFConstraint::GetLinearTargetVelocity() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PxDoFConstraint::SetLinearTargetVelocity(const Vector3 &vel) const
{
	// TODO
}
Vector3 pragma::physics::PxDoFConstraint::GetLinearMaxMotorForce() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PxDoFConstraint::SetLinearMaxMotorForce(const Vector3 &force)
{
	// TODO
}
float pragma::physics::PxDoFConstraint::GetLinearDamping() const
{
	// TODO
	return 0.f;
}
void pragma::physics::PxDoFConstraint::SetLinearDamping(float damping)
{
	// TODO
}
float pragma::physics::PxDoFConstraint::GetLinearLimitSoftness() const
{
	// TODO
	return 0.f;
}
void pragma::physics::PxDoFConstraint::SetLinearLimitSoftness(float softness)
{
	// TODO
}
Vector3 pragma::physics::PxDoFConstraint::GetLinearForceMixingFactor() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PxDoFConstraint::SetLinearForceMixingFactor(const Vector3 &factor)
{
	// TODO
}
Vector3 pragma::physics::PxDoFConstraint::GetLinearLimitErrorTolerance() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PxDoFConstraint::SetLinearLimitErrorTolerance(const Vector3 &tolerance)
{
	// TODO
}
Vector3 pragma::physics::PxDoFConstraint::GetLinearLimitForceMixingFactor() const
{
	// TODO
	return Vector3{};
}
void pragma::physics::PxDoFConstraint::SetLinearLimitForceMixingFactor(const Vector3 &factor)
{
	// TODO
}
float pragma::physics::PxDoFConstraint::GetLinearRestitutionFactor() const
{
	// TODO
	return 0.f;
}
void pragma::physics::PxDoFConstraint::SetLinearRestitutionFactor(float factor)
{
	// TODO
}
bool pragma::physics::PxDoFConstraint::IsLinearMotorEnabled(uint8_t axis) const
{
	// TODO
	return false;
}
void pragma::physics::PxDoFConstraint::SetLinearMotorEnabled(uint8_t axis,bool bEnabled)
{
	// TODO
}
Vector3 pragma::physics::PxDoFConstraint::GetCurrentLinearDifference() const
{
	// TODO
	return Vector3{};
}
Vector3 pragma::physics::PxDoFConstraint::GetCurrentLinearLimitError() const
{
	// TODO
	return Vector3{};
}
Vector3i pragma::physics::PxDoFConstraint::GetCurrentLinearLimit() const
{
	// TODO
	return Vector3i{};
}
Vector3 pragma::physics::PxDoFConstraint::GetCurrentLinearAccumulatedImpulse() const
{
	// TODO
	return Vector3{};
}

/////////////

pragma::physics::PxDoFSpringConstraint::PxDoFSpringConstraint(IEnvironment &env,PxUniquePtr<physx::PxJoint> c)
	: IDoFSpringConstraint{env},PxConstraint{env,std::move(c)},IConstraint{env}
{}
void pragma::physics::PxDoFSpringConstraint::CalculateTransforms()
{
	// TODO
}
void pragma::physics::PxDoFSpringConstraint::CalculateTransforms(const Transform &frameA,const Transform &frameB)
{
	// TODO
}
pragma::physics::Transform pragma::physics::PxDoFSpringConstraint::GetCalculatedTransformA() const
{
	// TODO
	return {};
}
pragma::physics::Transform pragma::physics::PxDoFSpringConstraint::GetCalculatedTransformB() const
{
	// TODO
	return {};
}
pragma::physics::Transform pragma::physics::PxDoFSpringConstraint::GetFrameOffsetA() const
{
	// TODO
	return {};
}
pragma::physics::Transform pragma::physics::PxDoFSpringConstraint::GetFrameOffsetB() const
{
	// TODO
	return {};
}
Vector3 pragma::physics::PxDoFSpringConstraint::GetAxis(pragma::Axis axisIndex) const
{
	// TODO
	return {};
}
double pragma::physics::PxDoFSpringConstraint::GetAngle(pragma::Axis axisIndex) const
{
	// TODO
	return {};
}
double pragma::physics::PxDoFSpringConstraint::GetRelativePivotPosition(pragma::Axis axisIndex) const
{
	// TODO
	return {};
}
void pragma::physics::PxDoFSpringConstraint::SetFrames(const Transform &frameA,const Transform &frameB)
{
	// TODO
}
void pragma::physics::PxDoFSpringConstraint::SetLinearLowerLimit(const Vector3 &linearLower)
{
	// TODO
}
Vector3 pragma::physics::PxDoFSpringConstraint::GetLinearLowerLimit() const
{
	// TODO
	return {};
}
void pragma::physics::PxDoFSpringConstraint::SetLinearUpperLimit(const Vector3 &linearUpper)
{
	// TODO
}
Vector3 pragma::physics::PxDoFSpringConstraint::GetLinearUpperLimit() const
{
	// TODO
	return {};
}
void pragma::physics::PxDoFSpringConstraint::SetAngularLowerLimit(const Vector3 &angularLower)
{
	// TODO
}
void pragma::physics::PxDoFSpringConstraint::SetAngularLowerLimitReversed(const Vector3 &angularLower)
{
	// TODO
}
Vector3 pragma::physics::PxDoFSpringConstraint::GetAngularLowerLimit() const
{
	// TODO
	return {};
}
Vector3 pragma::physics::PxDoFSpringConstraint::GetAngularLowerLimitReversed() const
{
	// TODO
	return {};
}
void pragma::physics::PxDoFSpringConstraint::SetAngularUpperLimit(const Vector3 &angularUpper)
{
	// TODO
}
void pragma::physics::PxDoFSpringConstraint::SetAngularUpperLimitReversed(const Vector3 &angularUpper)
{
	// TODO
}
Vector3 pragma::physics::PxDoFSpringConstraint::GetAngularUpperLimit() const
{
	// TODO
	return {};
}
Vector3 pragma::physics::PxDoFSpringConstraint::GetAngularUpperLimitReversed() const
{
	// TODO
	return {};
}
void pragma::physics::PxDoFSpringConstraint::SetLimit(AxisType type,pragma::Axis axis,double lo,double hi)
{
	// TODO
}
void pragma::physics::PxDoFSpringConstraint::SetLimitReversed(AxisType type,pragma::Axis axis,double lo,double hi)
{
	// TODO
}
bool pragma::physics::PxDoFSpringConstraint::IsLimited(AxisType type,pragma::Axis axis) const
{
	// TODO
	return {};
}
void pragma::physics::PxDoFSpringConstraint::SetRotationOrder(pragma::RotationOrder order)
{
	// TODO
}
pragma::RotationOrder pragma::physics::PxDoFSpringConstraint::GetRotationOrder() const
{
	// TODO
	return {};
}
void pragma::physics::PxDoFSpringConstraint::SetAxis(const Vector3 &axis1,const Vector3 &axis2)
{
	// TODO
}
void pragma::physics::PxDoFSpringConstraint::SetBounce(AxisType type,pragma::Axis axis,double bounce)
{
	// TODO
}
void pragma::physics::PxDoFSpringConstraint::EnableMotor(AxisType type,pragma::Axis axis,bool onOff)
{
	// TODO
}
void pragma::physics::PxDoFSpringConstraint::SetServo(AxisType type,pragma::Axis axis,bool onOff)
{
	// TODO
}
void pragma::physics::PxDoFSpringConstraint::SetTargetVelocity(AxisType type,pragma::Axis axis,double velocity)
{
	// TODO
}
void pragma::physics::PxDoFSpringConstraint::SetServoTarget(AxisType type,pragma::Axis axis,double target)
{
	// TODO
}
void pragma::physics::PxDoFSpringConstraint::SetMaxMotorForce(AxisType type,pragma::Axis axis,double force)
{
	// TODO
}
void pragma::physics::PxDoFSpringConstraint::EnableSpring(AxisType type,pragma::Axis axis,bool onOff)
{
	// TODO
}
void pragma::physics::PxDoFSpringConstraint::SetStiffness(AxisType type,pragma::Axis axis,double stiffness,bool limitIfNeeded)
{
	// TODO
}
void pragma::physics::PxDoFSpringConstraint::SetDamping(AxisType type,pragma::Axis axis,double damping,bool limitIfNeeded)
{
	// TODO
}
void pragma::physics::PxDoFSpringConstraint::SetEquilibriumPoint()
{
	// TODO
}
void pragma::physics::PxDoFSpringConstraint::SetEquilibriumPoint(AxisType type,pragma::Axis axis)
{
	// TODO
}
void pragma::physics::PxDoFSpringConstraint::SetEquilibriumPoint(AxisType type,pragma::Axis axis,double val)
{
	// TODO
}

void pragma::physics::PxDoFSpringConstraint::SetERP(AxisType type,pragma::Axis axis,double value)
{
	// TODO
}
double pragma::physics::PxDoFSpringConstraint::GetERP(AxisType type,pragma::Axis axis) const
{
	// TODO
	return {};
}
void pragma::physics::PxDoFSpringConstraint::SetStopERP(AxisType type,pragma::Axis axis,double value)
{
	// TODO
}
double pragma::physics::PxDoFSpringConstraint::GetStopERP(AxisType type,pragma::Axis axis) const
{
	// TODO
	return {};
}
void pragma::physics::PxDoFSpringConstraint::SetCFM(AxisType type,pragma::Axis axis,double value)
{
	// TODO
}
double pragma::physics::PxDoFSpringConstraint::GetCFM(AxisType type,pragma::Axis axis) const
{
	// TODO
	return {};
}
void pragma::physics::PxDoFSpringConstraint::SetStopCFM(AxisType type,pragma::Axis axis,double value)
{
	// TODO
}
double pragma::physics::PxDoFSpringConstraint::GetStopCFM(AxisType type,pragma::Axis axis) const
{
	// TODO
	return {};
}
