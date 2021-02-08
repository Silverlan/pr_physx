/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "pr_physx/constraint.hpp"
#include "pr_physx/environment.hpp"
#include "pr_physx/collision_object.hpp"

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
	GetInternalObject().setConstraintFlag(physx::PxConstraintFlag::eCOLLISION_ENABLED,b);
}
void pragma::physics::PhysXConstraint::SetEnabled(bool b)
{
	umath::set_flag(m_stateFlags,StateFlags::Enabled,b);
	if(IsBroken())
		return;
	GetInternalObject().setConstraintFlag(physx::PxConstraintFlag::eBROKEN,b);
}
bool pragma::physics::PhysXConstraint::IsEnabled() const
{
	return umath::is_flag_set(m_stateFlags,StateFlags::Enabled);
}
void pragma::physics::PhysXConstraint::Break()
{
	umath::set_flag(m_stateFlags,StateFlags::Broken);
	GetInternalObject().setConstraintFlag(physx::PxConstraintFlag::eBROKEN,true);
}
bool pragma::physics::PhysXConstraint::IsBroken() const
{
	return umath::is_flag_set(m_stateFlags,StateFlags::Broken);
}
pragma::physics::IRigidBody *pragma::physics::PhysXConstraint::GetSourceActor()
{
	physx::PxRigidActor *pxActor0;
	physx::PxRigidActor *pxActor1;
	GetInternalObject().getActors(pxActor0,pxActor1);
	if(pxActor0 == nullptr)
		return nullptr;
	auto *actor0 = GetPxEnv().GetCollisionObject(*pxActor0);
	return (actor0 && actor0->IsRigid()) ? static_cast<PhysXRigidBody*>(actor0) : nullptr;
}
pragma::physics::IRigidBody *pragma::physics::PhysXConstraint::GetTargetActor()
{
	physx::PxRigidActor *pxActor0;
	physx::PxRigidActor *pxActor1;
	GetInternalObject().getActors(pxActor0,pxActor1);
	if(pxActor1 == nullptr)
		return nullptr;
	auto *actor1 = GetPxEnv().GetCollisionObject(*pxActor1);
	return (actor1 && actor1->IsRigid()) ? static_cast<PhysXRigidBody*>(actor1) : nullptr;
}
float pragma::physics::PhysXConstraint::GetBreakForce() const
{
	physx::PxReal force,torque;
	GetInternalObject().getBreakForce(force,torque);
	return GetPxEnv().FromPhysXLength(force);
}
void pragma::physics::PhysXConstraint::SetBreakForce(float force)
{
	physx::PxReal oldForce,torque;
	GetInternalObject().getBreakForce(oldForce,torque);
	GetInternalObject().setBreakForce(GetPxEnv().ToPhysXLength(force),torque);
}
float pragma::physics::PhysXConstraint::GetBreakTorque() const
{
	physx::PxReal force,torque;
	GetInternalObject().getBreakForce(force,torque);
	return GetPxEnv().FromPhysXTorque(torque);
}
void pragma::physics::PhysXConstraint::SetBreakTorque(float torque)
{
	physx::PxReal force,oldTorque;
	GetInternalObject().getBreakForce(force,oldTorque);
	GetInternalObject().setBreakForce(force,GetPxEnv().ToPhysXTorque(torque));
}
void pragma::physics::PhysXConstraint::SetSoftness(float softness) {}
void pragma::physics::PhysXConstraint::SetDamping(float damping) {}
void pragma::physics::PhysXConstraint::SetRestitution(float restitution) {}

float pragma::physics::PhysXConstraint::GetSoftness() const {return 0.f;}
float pragma::physics::PhysXConstraint::GetDamping() const {return 0.f;}
float pragma::physics::PhysXConstraint::GetRestitution() const {return 0.f;}

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
{
	auto limit = GetInternalObject().getLimit();
	m_lowerLimit = limit.lower;
	m_upperLimit = limit.upper;
	m_softness = limit.stiffness;
	m_damping = limit.damping;
	m_restitution = limit.restitution;
}
physx::PxRevoluteJoint &pragma::physics::PhysXHingeConstraint::GetInternalObject() const {return static_cast<physx::PxRevoluteJoint&>(PhysXConstraint::GetInternalObject());}
void pragma::physics::PhysXHingeConstraint::UpdateLimits()
{
	auto limit = GetInternalObject().getLimit();
	limit.lower = m_lowerLimit;
	limit.upper = m_upperLimit;
	limit.stiffness = m_softness;
	limit.damping = m_damping;
	limit.restitution = m_restitution;
	GetInternalObject().setLimit(limit);
}
void pragma::physics::PhysXHingeConstraint::SetLimit(umath::Radian lowerLimit,umath::Radian upperLimit)
{
	m_lowerLimit = lowerLimit;
	m_upperLimit = upperLimit;
	UpdateLimits();
	GetInternalObject().setRevoluteJointFlag(physx::PxRevoluteJointFlag::eLIMIT_ENABLED,true);
}
std::pair<umath::Radian,umath::Radian> pragma::physics::PhysXHingeConstraint::GetLimit() const
{
	return {m_lowerLimit,m_upperLimit};
}

void pragma::physics::PhysXHingeConstraint::SetSoftness(float softness)
{
	m_softness = softness;
	UpdateLimits();
}
void pragma::physics::PhysXHingeConstraint::SetDamping(float damping)
{
	m_damping = damping;
	UpdateLimits();
}
void pragma::physics::PhysXHingeConstraint::DisableLimit()
{
	GetInternalObject().setRevoluteJointFlag(physx::PxRevoluteJointFlag::eLIMIT_ENABLED,false);
}
float pragma::physics::PhysXHingeConstraint::GetSoftness() const
{
	return m_softness;
}
float pragma::physics::PhysXHingeConstraint::GetDamping() const
{
	return m_damping;
}
void pragma::physics::PhysXHingeConstraint::SetRestitution(float restitution)
{
	m_restitution = restitution;
	UpdateLimits();
}
float pragma::physics::PhysXHingeConstraint::GetRestitution() const {return GetInternalObject().getLimit().restitution;}

/////////////

pragma::physics::PhysXSliderConstraint::PhysXSliderConstraint(IEnvironment &env,PhysXUniquePtr<physx::PxJoint> c)
	: ISliderConstraint{env},PhysXConstraint{env,std::move(c)},IConstraint{env}
{
	auto limit = GetInternalObject().getLimit();
	m_softness = limit.stiffness;
	m_damping = limit.damping;
	m_restitution = limit.restitution;
	m_lowerLimit = limit.lower;
	m_upperLimit = limit.upper;
}
physx::PxPrismaticJoint &pragma::physics::PhysXSliderConstraint::GetInternalObject() const {return static_cast<physx::PxPrismaticJoint&>(PhysXConstraint::GetInternalObject());}
void pragma::physics::PhysXSliderConstraint::UpdateLimits()
{
	auto limit = GetInternalObject().getLimit();
	limit.lower = m_lowerLimit;
	limit.upper = m_upperLimit;
	limit.stiffness = m_softness;
	limit.damping = m_damping;
	limit.restitution = m_restitution;
	GetInternalObject().setLimit(limit);
}
void pragma::physics::PhysXSliderConstraint::SetLimit(float lowerLimit,float upperLimit)
{
	m_lowerLimit = lowerLimit;
	m_upperLimit = upperLimit;
	UpdateLimits();
	GetInternalObject().setPrismaticJointFlag(physx::PxPrismaticJointFlag::eLIMIT_ENABLED,true);
}
void pragma::physics::PhysXSliderConstraint::DisableLimit()
{
	GetInternalObject().setPrismaticJointFlag(physx::PxPrismaticJointFlag::eLIMIT_ENABLED,false);
}
void pragma::physics::PhysXSliderConstraint::SetSoftness(float softness)
{
	m_softness = softness;
	UpdateLimits();
}
void pragma::physics::PhysXSliderConstraint::SetDamping(float damping)
{
	m_damping = damping;
	UpdateLimits();
}
void pragma::physics::PhysXSliderConstraint::SetRestitution(float restitution)
{
	m_restitution = restitution;
	UpdateLimits();
}
std::pair<float,float> pragma::physics::PhysXSliderConstraint::GetLimit() const
{
	return {
		GetPxEnv().FromPhysXLength(GetInternalObject().getLimit().lower),
		GetPxEnv().FromPhysXLength(GetInternalObject().getLimit().upper)
	};
}
float pragma::physics::PhysXSliderConstraint::GetSoftness() const {return GetInternalObject().getLimit().stiffness;}
float pragma::physics::PhysXSliderConstraint::GetDamping() const {return GetInternalObject().getLimit().damping;}
float pragma::physics::PhysXSliderConstraint::GetRestitution() const {return GetInternalObject().getLimit().restitution;}

/////////////

pragma::physics::PhysXConeTwistConstraint::PhysXConeTwistConstraint(IEnvironment &env,PhysXUniquePtr<physx::PxJoint> c,const physx::PxTransform &pose0,const physx::PxTransform &pose1)
	: IConeTwistConstraint{env},PhysXConstraint{env,std::move(c)},IConstraint{env},
	m_localPoses{pose0,pose1}
{
	auto &joint = GetInternalObject();
	/*auto limit = joint.getLimitCone();
	limit.yAngle = umath::deg_to_rad(40.f);
	limit.zAngle = umath::deg_to_rad(40.f);
	limit.restitution = 1.f;
	joint.setLimitCone(limit);
	GetInternalObject().setSphericalJointFlag(physx::PxSphericalJointFlag::eLIMIT_ENABLED,true);*/
	auto swingLimit = joint.getSwingLimit();
	m_softness = swingLimit.stiffness;
	m_damping = swingLimit.damping;
	m_restitution = 1.f;//swingLimit.restitution;

	joint.setMotion(physx::PxD6Axis::eSWING1,physx::PxD6Motion::eLIMITED);
	joint.setMotion(physx::PxD6Axis::eSWING2,physx::PxD6Motion::eLOCKED);
	joint.setMotion(physx::PxD6Axis::eTWIST,physx::PxD6Motion::eLOCKED);
	UpdateLimits();

	SetLimit(Vector3{umath::deg_to_rad(-5),0.f,0.f},Vector3{umath::deg_to_rad(5),0.f,0.f});
}
physx::PxD6Joint &pragma::physics::PhysXConeTwistConstraint::GetInternalObject() const {return static_cast<physx::PxD6Joint&>(PhysXConstraint::GetInternalObject());}
void pragma::physics::PhysXConeTwistConstraint::UpdateLocalPoses()
{
	auto centerLimits = Vector2{
		(m_lowerLimits.x +m_upperLimits.x) /2.f,
		(m_lowerLimits.y +m_upperLimits.y) /2.f
	};
	auto pose0 = m_localPoses.front();
	pose0.q *= uquat::create_px(uquat::create(EulerAngles{centerLimits.x,centerLimits.y,0.f}));
	
	auto &joint = GetInternalObject();
	joint.setLocalPose(physx::PxJointActorIndex::eACTOR0,pose0);
	joint.setLocalPose(physx::PxJointActorIndex::eACTOR1,m_localPoses.at(1));
}
void pragma::physics::PhysXConeTwistConstraint::UpdateLimits()
{
	Vector2 swingSpan = {
		m_upperLimits.x -m_lowerLimits.x,
		m_upperLimits.y -m_lowerLimits.y
	};
	auto swingLimit = GetInternalObject().getSwingLimit();
	swingLimit.yAngle = swingSpan.x;
	swingLimit.zAngle = swingSpan.y;
	swingLimit.stiffness = m_softness;
	swingLimit.damping = m_damping;
	swingLimit.restitution = m_restitution;
	GetInternalObject().setSwingLimit(swingLimit);

	auto twistLimit = GetInternalObject().getTwistLimit();
	twistLimit.lower = -m_lowerLimits.z;
	twistLimit.upper = m_lowerLimits.z;
	twistLimit.stiffness = m_softness;
	twistLimit.damping = m_damping;
	twistLimit.restitution = m_restitution;
	GetInternalObject().setTwistLimit(twistLimit);
}
void pragma::physics::PhysXConeTwistConstraint::SetLimit(float swingSpan1,float swingSpan2,float twistSpan)
{
	/*m_swingSpan1 = swingSpan1;
	m_swingSpan2 = swingSpan2;
	m_twistLimitLower = -twistSpan;
	m_twistLimitUpper = twistSpan; // TODO
	UpdateLimits();*/
}
void pragma::physics::PhysXConeTwistConstraint::SetLimit(const Vector3 &lowerLimits,const Vector3 &upperLimits)
{
	m_lowerLimits = lowerLimits;
	m_upperLimits = upperLimits;
	UpdateLimits();
}
void pragma::physics::PhysXConeTwistConstraint::GetLimit(float &outSwingSpan1,float &outSwingSpan2,float &outTwistSpan)
{
	//outSwingSpan1 = m_swingSpan1;
	//outSwingSpan2 = m_swingSpan2;
	//outTwistSpan = m_twistLimitUpper; // TODO
}
void pragma::physics::PhysXConeTwistConstraint::SetSoftness(float softness)
{
	auto swingLimit = GetInternalObject().getSwingLimit();
	auto twistLimit = GetInternalObject().getTwistLimit();
	m_softness = softness;
	UpdateLimits();
}
void pragma::physics::PhysXConeTwistConstraint::SetDamping(float damping)
{
	auto swingLimit = GetInternalObject().getSwingLimit();
	auto twistLimit = GetInternalObject().getTwistLimit();
	m_damping = damping;
	UpdateLimits();
}
void pragma::physics::PhysXConeTwistConstraint::SetRestitution(float restitution)
{
	auto swingLimit = GetInternalObject().getSwingLimit();
	auto twistLimit = GetInternalObject().getTwistLimit();
	m_restitution = restitution;
	UpdateLimits();
}

float pragma::physics::PhysXConeTwistConstraint::GetSoftness() const {return m_softness;}
float pragma::physics::PhysXConeTwistConstraint::GetDamping() const {return m_damping;}
float pragma::physics::PhysXConeTwistConstraint::GetRestitution() const {return m_restitution;}

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
void pragma::physics::PhysXDoFSpringConstraint::CalculateTransforms(const umath::Transform &frameA,const umath::Transform &frameB)
{
	// TODO
}
umath::Transform pragma::physics::PhysXDoFSpringConstraint::GetCalculatedTransformA() const
{
	// TODO
	return {};
}
umath::Transform pragma::physics::PhysXDoFSpringConstraint::GetCalculatedTransformB() const
{
	// TODO
	return {};
}
umath::Transform pragma::physics::PhysXDoFSpringConstraint::GetFrameOffsetA() const
{
	// TODO
	return {};
}
umath::Transform pragma::physics::PhysXDoFSpringConstraint::GetFrameOffsetB() const
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
void pragma::physics::PhysXDoFSpringConstraint::SetFrames(const umath::Transform &frameA,const umath::Transform &frameB)
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
