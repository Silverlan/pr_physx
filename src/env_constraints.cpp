#include "pr_physx/environment.hpp"
#include "pr_physx/constraint.hpp"
#include "pr_physx/collision_object.hpp"
#include <pragma/networkstate/networkstate.h>

util::TSharedHandle<pragma::physics::IFixedConstraint> pragma::physics::PxEnvironment::CreateFixedConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB)
{
	physx::PxTransform tA {uvec::create_px(pivotA),uquat::create_px(rotA)};
	physx::PxTransform tB {uvec::create_px(pivotB),uquat::create_px(rotB)};
	auto fixedJoint = px_create_unique_ptr<physx::PxJoint>(physx::PxFixedJointCreate(GetPhysics(),&ToBtType(a).GetInternalObject(),tA,&ToBtType(b).GetInternalObject(),tB));
	auto r = util::shared_handle_cast<PxFixedConstraint,IFixedConstraint>(CreateSharedHandle<PxFixedConstraint>(*this,std::move(fixedJoint)));
	AddConstraint(*r);
	return r;
}
util::TSharedHandle<pragma::physics::IBallSocketConstraint> pragma::physics::PxEnvironment::CreateBallSocketConstraint(IRigidBody &a,const Vector3 &pivotA,IRigidBody &b,const Vector3 &pivotB)
{
	physx::PxTransform tA {uvec::create_px(pivotA)};
	physx::PxTransform tB {uvec::create_px(pivotB)};
	auto sphericalJoint = px_create_unique_ptr<physx::PxJoint>(physx::PxSphericalJointCreate(GetPhysics(),&ToBtType(a).GetInternalObject(),tA,&ToBtType(b).GetInternalObject(),tB));
	auto r = util::shared_handle_cast<PxBallSocketConstraint,IBallSocketConstraint>(CreateSharedHandle<PxBallSocketConstraint>(*this,std::move(sphericalJoint)));
	AddConstraint(*r);
	return r;
}
util::TSharedHandle<pragma::physics::IHingeConstraint> pragma::physics::PxEnvironment::CreateHingeConstraint(IRigidBody &a,const Vector3 &pivotA,IRigidBody &b,const Vector3 &pivotB,const Vector3 &axis)
{
	physx::PxTransform tA {uvec::create_px(pivotA)};
	physx::PxTransform tB {uvec::create_px(pivotB)};
	auto hingeJoint = px_create_unique_ptr<physx::PxJoint>(physx::PxRevoluteJointCreate(GetPhysics(),&ToBtType(a).GetInternalObject(),tA,&ToBtType(b).GetInternalObject(),tB));
	// TODO: Specify axis?
	auto r = util::shared_handle_cast<PxHingeConstraint,IHingeConstraint>(CreateSharedHandle<PxHingeConstraint>(*this,std::move(hingeJoint)));
	AddConstraint(*r);
	return r;
}
util::TSharedHandle<pragma::physics::ISliderConstraint> pragma::physics::PxEnvironment::CreateSliderConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB)
{
	physx::PxTransform tA {uvec::create_px(pivotA),uquat::create_px(rotA)};
	physx::PxTransform tB {uvec::create_px(pivotB),uquat::create_px(rotB)};
	auto prismaticJoint = px_create_unique_ptr<physx::PxJoint>(physx::PxPrismaticJointCreate(GetPhysics(),&ToBtType(a).GetInternalObject(),tA,&ToBtType(b).GetInternalObject(),tB));
	auto r = util::shared_handle_cast<PxSliderConstraint,ISliderConstraint>(CreateSharedHandle<PxSliderConstraint>(*this,std::move(prismaticJoint)));
	AddConstraint(*r);
	return r;
}
util::TSharedHandle<pragma::physics::IConeTwistConstraint> pragma::physics::PxEnvironment::CreateConeTwistConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB)
{
	physx::PxTransform tA {uvec::create_px(pivotA),uquat::create_px(rotA)};
	physx::PxTransform tB {uvec::create_px(pivotB),uquat::create_px(rotB)};
	auto sphericalJoint = px_create_unique_ptr<physx::PxJoint>(physx::PxSphericalJointCreate(GetPhysics(),&ToBtType(a).GetInternalObject(),tA,&ToBtType(b).GetInternalObject(),tB));
	auto r = util::shared_handle_cast<PxConeTwistConstraint,IConeTwistConstraint>(CreateSharedHandle<PxConeTwistConstraint>(*this,std::move(sphericalJoint)));
	AddConstraint(*r);
	return r;
}
util::TSharedHandle<pragma::physics::IDoFConstraint> pragma::physics::PxEnvironment::CreateDoFConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB)
{
	physx::PxTransform tA {uvec::create_px(pivotA),uquat::create_px(rotA)};
	physx::PxTransform tB {uvec::create_px(pivotB),uquat::create_px(rotB)};
	auto d6Joint = px_create_unique_ptr<physx::PxJoint>(physx::PxD6JointCreate(GetPhysics(),&ToBtType(a).GetInternalObject(),tA,&ToBtType(b).GetInternalObject(),tB));
	auto r = util::shared_handle_cast<PxDoFConstraint,IDoFConstraint>(CreateSharedHandle<PxDoFConstraint>(*this,std::move(d6Joint)));
	AddConstraint(*r);
	return r;
}
util::TSharedHandle<pragma::physics::IDoFSpringConstraint> pragma::physics::PxEnvironment::CreateDoFSpringConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB)
{
	// TODO?
	return nullptr;
}
