#include "pr_physx/common.hpp"

physx::PxVec3 uvec::create_px(const Vector3 &v)
{
	return physx::PxVec3{v.x,v.y,v.z};
}
Vector3 uvec::create(const physx::PxVec3 &v)
{
	return Vector3{v.x,v.y,v.z};
}

physx::PxQuat uquat::create_px(const Quat &v)
{
	return physx::PxQuat{v.x,v.y,v.z,v.w};
}
Quat uquat::create(const physx::PxQuat &v)
{
	return Quat{v.w,v.x,v.y,v.z};
}
