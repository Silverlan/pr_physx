#include "pr_physx/material.hpp"
#include "pr_physx/common.hpp"

pragma::physics::PhysXMaterial &pragma::physics::PhysXMaterial::GetMaterial(IMaterial &o)
{
	return *static_cast<PhysXMaterial*>(o.GetUserData());
}
const pragma::physics::PhysXMaterial &pragma::physics::PhysXMaterial::GetMaterial(const IMaterial &o) {return GetMaterial(const_cast<IMaterial&>(o));}
pragma::physics::PhysXMaterial::PhysXMaterial(IEnvironment &env,PhysXUniquePtr<physx::PxMaterial> material)
	: IMaterial{env},m_material{std::move(material)}
{
	SetUserData(this);
}
void pragma::physics::PhysXMaterial::Initialize()
{
	IMaterial::Initialize();
	GetInternalObject().userData = this;
}
const physx::PxMaterial &pragma::physics::PhysXMaterial::GetInternalObject() const {return const_cast<PhysXMaterial*>(this)->GetInternalObject();}
physx::PxMaterial &pragma::physics::PhysXMaterial::GetInternalObject() {return *m_material;}
float pragma::physics::PhysXMaterial::GetStaticFriction() const {return GetInternalObject().getStaticFriction();}
void pragma::physics::PhysXMaterial::SetStaticFriction(float friction) {GetInternalObject().setStaticFriction(friction);}
float pragma::physics::PhysXMaterial::GetDynamicFriction() const {return GetInternalObject().getDynamicFriction();}
void pragma::physics::PhysXMaterial::SetDynamicFriction(float friction) {GetInternalObject().setDynamicFriction(friction);}
float pragma::physics::PhysXMaterial::GetRestitution() const {return GetInternalObject().getRestitution();}
void pragma::physics::PhysXMaterial::SetRestitution(float restitution) {GetInternalObject().setRestitution(umath::clamp(restitution,0.f,1.f));}
