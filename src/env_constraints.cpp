#include "pr_physx/environment.hpp"
#include "pr_physx/constraint.hpp"
#include "pr_physx/collision_object.hpp"
#include <pragma/networkstate/networkstate.h>

util::TSharedHandle<pragma::physics::IFixedConstraint> pragma::physics::PhysXEnvironment::CreateFixedConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB)
{
	physx::PxTransform tA {uvec::create_px(pivotA),uquat::create_px(rotA)};
	physx::PxTransform tB {uvec::create_px(pivotB),uquat::create_px(rotB)};
	auto fixedJoint = px_create_unique_ptr<physx::PxJoint>(physx::PxFixedJointCreate(GetPhysics(),&ToBtType(a).GetInternalObject(),tA,&ToBtType(b).GetInternalObject(),tB));
	auto r = util::shared_handle_cast<PhysXFixedConstraint,IFixedConstraint>(CreateSharedHandle<PhysXFixedConstraint>(*this,std::move(fixedJoint)));
	AddConstraint(*r);
	return r;
}
util::TSharedHandle<pragma::physics::IBallSocketConstraint> pragma::physics::PhysXEnvironment::CreateBallSocketConstraint(IRigidBody &a,const Vector3 &pivotA,IRigidBody &b,const Vector3 &pivotB)
{
	physx::PxTransform tA {uvec::create_px(pivotA)};
	physx::PxTransform tB {uvec::create_px(pivotB)};
	auto sphericalJoint = px_create_unique_ptr<physx::PxJoint>(physx::PxSphericalJointCreate(GetPhysics(),&ToBtType(a).GetInternalObject(),tA,&ToBtType(b).GetInternalObject(),tB));
	auto r = util::shared_handle_cast<PhysXBallSocketConstraint,IBallSocketConstraint>(CreateSharedHandle<PhysXBallSocketConstraint>(*this,std::move(sphericalJoint)));
	AddConstraint(*r);
	return r;
}
util::TSharedHandle<pragma::physics::IHingeConstraint> pragma::physics::PhysXEnvironment::CreateHingeConstraint(IRigidBody &a,const Vector3 &pivotA,IRigidBody &b,const Vector3 &pivotB,const Vector3 &axis)
{

	/*m_rbAFrame.getOrigin() = pivotInA;

	// since no frame is given, assume this to be zero angle and just pick rb transform axis
	btVector3 rbAxisA1 = rbA.getCenterOfMassTransform().getBasis().getColumn(0);

	btVector3 rbAxisA2;
	btScalar projection = axisInA.dot(rbAxisA1);
	if (projection >= 1.0f - SIMD_EPSILON)
	{
		rbAxisA1 = -rbA.getCenterOfMassTransform().getBasis().getColumn(2);
		rbAxisA2 = rbA.getCenterOfMassTransform().getBasis().getColumn(1);
	}
	else if (projection <= -1.0f + SIMD_EPSILON)
	{
		rbAxisA1 = rbA.getCenterOfMassTransform().getBasis().getColumn(2);
		rbAxisA2 = rbA.getCenterOfMassTransform().getBasis().getColumn(1);
	}
	else
	{
		rbAxisA2 = axisInA.cross(rbAxisA1);
		rbAxisA1 = rbAxisA2.cross(axisInA);
	}

	m_rbAFrame.getBasis().setValue(rbAxisA1.getX(), rbAxisA2.getX(), axisInA.getX(),
		rbAxisA1.getY(), rbAxisA2.getY(), axisInA.getY(),
		rbAxisA1.getZ(), rbAxisA2.getZ(), axisInA.getZ());

	btQuaternion rotationArc = shortestArcQuat(axisInA, axisInB);
	btVector3 rbAxisB1 = quatRotate(rotationArc, rbAxisA1);
	btVector3 rbAxisB2 = axisInB.cross(rbAxisB1);

	m_rbBFrame.getOrigin() = pivotInB;
	m_rbBFrame.getBasis().setValue(rbAxisB1.getX(), rbAxisB2.getX(), axisInB.getX(),
		rbAxisB1.getY(), rbAxisB2.getY(), axisInB.getY(),
		rbAxisB1.getZ(), rbAxisB2.getZ(), axisInB.getZ());
		*/

	auto right = uvec::get_perpendicular(axis);
	auto up = uvec::cross(axis,right);
	uvec::normalize(&up);
	auto rot = uquat::identity();//uquat::create(axis,right,up);
	//rot /= rot.length();

	physx::PxTransform tA {uvec::create_px(pivotA),uquat::create_px(rot)};
	physx::PxTransform tB {uvec::create_px(pivotB),uquat::create_px(rot)};
	auto hingeJoint = px_create_unique_ptr<physx::PxJoint>(physx::PxRevoluteJointCreate(GetPhysics(),&ToBtType(a).GetInternalObject(),tA,&ToBtType(b).GetInternalObject(),tB));
	// TODO: Specify axis?
	auto r = util::shared_handle_cast<PhysXHingeConstraint,IHingeConstraint>(CreateSharedHandle<PhysXHingeConstraint>(*this,std::move(hingeJoint)));
	AddConstraint(*r);
	return r;
}
util::TSharedHandle<pragma::physics::ISliderConstraint> pragma::physics::PhysXEnvironment::CreateSliderConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB)
{
	physx::PxTransform tA {uvec::create_px(pivotA),uquat::create_px(rotA)};
	physx::PxTransform tB {uvec::create_px(pivotB),uquat::create_px(rotB)};
	auto prismaticJoint = px_create_unique_ptr<physx::PxJoint>(physx::PxPrismaticJointCreate(GetPhysics(),&ToBtType(a).GetInternalObject(),tA,&ToBtType(b).GetInternalObject(),tB));
	auto r = util::shared_handle_cast<PhysXSliderConstraint,ISliderConstraint>(CreateSharedHandle<PhysXSliderConstraint>(*this,std::move(prismaticJoint)));
	AddConstraint(*r);
	return r;
}
util::TSharedHandle<pragma::physics::IConeTwistConstraint> pragma::physics::PhysXEnvironment::CreateConeTwistConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB)
{
	physx::PxTransform tA {uvec::create_px(pivotA),uquat::create_px(rotA)};
	physx::PxTransform tB {uvec::create_px(pivotB),uquat::create_px(rotB)};
	auto sphericalJoint = px_create_unique_ptr<physx::PxJoint>(physx::PxSphericalJointCreate(GetPhysics(),&ToBtType(a).GetInternalObject(),tA,&ToBtType(b).GetInternalObject(),tB));
	auto r = util::shared_handle_cast<PhysXConeTwistConstraint,IConeTwistConstraint>(CreateSharedHandle<PhysXConeTwistConstraint>(*this,std::move(sphericalJoint)));
	AddConstraint(*r);
	return r;
}
util::TSharedHandle<pragma::physics::IDoFConstraint> pragma::physics::PhysXEnvironment::CreateDoFConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB)
{
	physx::PxTransform tA {uvec::create_px(pivotA),uquat::create_px(rotA)};
	physx::PxTransform tB {uvec::create_px(pivotB),uquat::create_px(rotB)};
	auto d6Joint = px_create_unique_ptr<physx::PxJoint>(physx::PxD6JointCreate(GetPhysics(),&ToBtType(a).GetInternalObject(),tA,&ToBtType(b).GetInternalObject(),tB));
	auto r = util::shared_handle_cast<PhysXDoFConstraint,IDoFConstraint>(CreateSharedHandle<PhysXDoFConstraint>(*this,std::move(d6Joint)));
	AddConstraint(*r);
	return r;
}
util::TSharedHandle<pragma::physics::IDoFSpringConstraint> pragma::physics::PhysXEnvironment::CreateDoFSpringConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB)
{
	// TODO?
	return nullptr;
}
