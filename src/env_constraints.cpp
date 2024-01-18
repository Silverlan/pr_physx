/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "pr_physx/environment.hpp"
#include "pr_physx/constraint.hpp"
#include "pr_physx/collision_object.hpp"
#include <pragma/networkstate/networkstate.h>

util::TSharedHandle<pragma::physics::IFixedConstraint> pragma::physics::PhysXEnvironment::CreateFixedConstraint(IRigidBody &a, const Vector3 &pivotA, const Quat &rotA, IRigidBody &b, const Vector3 &pivotB, const Quat &rotB)
{
	physx::PxTransform tA {uvec::create_px(pivotA), uquat::create_px(rotA)};
	physx::PxTransform tB {uvec::create_px(pivotB), uquat::create_px(rotB)};
	auto fixedJoint = px_create_unique_ptr<physx::PxJoint>(physx::PxFixedJointCreate(GetPhysics(), &ToBtType(a).GetInternalObject(), tA, &ToBtType(b).GetInternalObject(), tB));
	auto r = util::shared_handle_cast<PhysXFixedConstraint, IFixedConstraint>(CreateSharedHandle<PhysXFixedConstraint>(*this, std::move(fixedJoint)));
	AddConstraint(*r);
	return r;
}
util::TSharedHandle<pragma::physics::IBallSocketConstraint> pragma::physics::PhysXEnvironment::CreateBallSocketConstraint(IRigidBody &a, const Vector3 &pivotA, IRigidBody &b, const Vector3 &pivotB)
{
	physx::PxTransform tA {uvec::create_px(pivotA)};
	physx::PxTransform tB {uvec::create_px(pivotB)};
	auto sphericalJoint = px_create_unique_ptr<physx::PxJoint>(physx::PxSphericalJointCreate(GetPhysics(), &ToBtType(a).GetInternalObject(), tA, &ToBtType(b).GetInternalObject(), tB));
	auto r = util::shared_handle_cast<PhysXBallSocketConstraint, IBallSocketConstraint>(CreateSharedHandle<PhysXBallSocketConstraint>(*this, std::move(sphericalJoint)));
	AddConstraint(*r);
	return r;
}
util::TSharedHandle<pragma::physics::IHingeConstraint> pragma::physics::PhysXEnvironment::CreateHingeConstraint(IRigidBody &a, const Vector3 &pivotA, IRigidBody &b, const Vector3 &pivotB, const Vector3 &axis)
{
	auto rot = uquat::create(axis, umath::deg_to_rad(90.f));
	constexpr Quat rot90DegPitch {0.70710676908493, 0.70710676908493, 0, 0};
	rot = rot90DegPitch * rot;

	physx::PxTransform tA {uvec::create_px(pivotA), uquat::create_px(rot)};
	physx::PxTransform tB {uvec::create_px(pivotB), uquat::create_px(rot)};
	auto hingeJoint = px_create_unique_ptr<physx::PxJoint>(physx::PxRevoluteJointCreate(GetPhysics(), &ToBtType(a).GetInternalObject(), tA, &ToBtType(b).GetInternalObject(), tB));
	auto r = util::shared_handle_cast<PhysXHingeConstraint, IHingeConstraint>(CreateSharedHandle<PhysXHingeConstraint>(*this, std::move(hingeJoint)));
	AddConstraint(*r);
	return r;
}
util::TSharedHandle<pragma::physics::ISliderConstraint> pragma::physics::PhysXEnvironment::CreateSliderConstraint(IRigidBody &a, const Vector3 &pivotA, const Quat &rotA, IRigidBody &b, const Vector3 &pivotB, const Quat &rotB)
{
	physx::PxTransform tA {uvec::create_px(pivotA), uquat::create_px(rotA)};
	physx::PxTransform tB {uvec::create_px(pivotB), uquat::create_px(rotB)};
	auto prismaticJoint = px_create_unique_ptr<physx::PxJoint>(physx::PxPrismaticJointCreate(GetPhysics(), &ToBtType(a).GetInternalObject(), tA, &ToBtType(b).GetInternalObject(), tB));
	auto r = util::shared_handle_cast<PhysXSliderConstraint, ISliderConstraint>(CreateSharedHandle<PhysXSliderConstraint>(*this, std::move(prismaticJoint)));
	AddConstraint(*r);
	return r;
}
util::TSharedHandle<pragma::physics::IConeTwistConstraint> pragma::physics::PhysXEnvironment::CreateConeTwistConstraint(IRigidBody &a, const Vector3 &pivotA, const Quat &rotA, IRigidBody &b, const Vector3 &pivotB, const Quat &rotB)
{
	physx::PxTransform tA {uvec::create_px(pivotA), uquat::create_px(rotA)};
	physx::PxTransform tB {uvec::create_px(pivotB), uquat::create_px(rotB)};
	auto sphericalJoint = px_create_unique_ptr<physx::PxJoint>(physx::PxD6JointCreate(GetPhysics(), &ToBtType(a).GetInternalObject(), tA, &ToBtType(b).GetInternalObject(), tB));
	auto r = util::shared_handle_cast<PhysXConeTwistConstraint, IConeTwistConstraint>(CreateSharedHandle<PhysXConeTwistConstraint>(*this, std::move(sphericalJoint), tA, tB));
	AddConstraint(*r);

	//r->SetLimit(umath::deg_to_rad(20.f),umath::deg_to_rad(20.f),umath::deg_to_rad(20.f));
	auto &joint = static_cast<physx::PxD6Joint &>(util::shared_handle_cast<IConeTwistConstraint, PhysXConeTwistConstraint>(r)->GetInternalObject());
	joint.setMotion(physx::PxD6Axis::eSWING1, physx::PxD6Motion::eLOCKED);
	joint.setMotion(physx::PxD6Axis::eSWING2, physx::PxD6Motion::eLIMITED);
	joint.setMotion(physx::PxD6Axis::eTWIST, physx::PxD6Motion::eLOCKED);

	return r;
}
util::TSharedHandle<pragma::physics::IDoFConstraint> pragma::physics::PhysXEnvironment::CreateDoFConstraint(IRigidBody &a, const Vector3 &pivotA, const Quat &rotA, IRigidBody &b, const Vector3 &pivotB, const Quat &rotB)
{
	physx::PxTransform tA {uvec::create_px(pivotA), uquat::create_px(rotA)};
	physx::PxTransform tB {uvec::create_px(pivotB), uquat::create_px(rotB)};
	auto d6Joint = px_create_unique_ptr<physx::PxJoint>(physx::PxD6JointCreate(GetPhysics(), &ToBtType(a).GetInternalObject(), tA, &ToBtType(b).GetInternalObject(), tB));
	auto r = util::shared_handle_cast<PhysXDoFConstraint, IDoFConstraint>(CreateSharedHandle<PhysXDoFConstraint>(*this, std::move(d6Joint)));
	AddConstraint(*r);

	auto &joint = static_cast<physx::PxD6Joint &>(util::shared_handle_cast<IDoFConstraint, PhysXDoFConstraint>(r)->GetInternalObject());
	joint.setMotion(physx::PxD6Axis::eSWING1, physx::PxD6Motion::eFREE);
	joint.setMotion(physx::PxD6Axis::eSWING2, physx::PxD6Motion::eFREE);
	joint.setMotion(physx::PxD6Axis::eTWIST, physx::PxD6Motion::eFREE);
	// TODO

	return r;
}
util::TSharedHandle<pragma::physics::IDoFSpringConstraint> pragma::physics::PhysXEnvironment::CreateDoFSpringConstraint(IRigidBody &a, const Vector3 &pivotA, const Quat &rotA, IRigidBody &b, const Vector3 &pivotB, const Quat &rotB)
{
	// TODO?
	return nullptr;
}
