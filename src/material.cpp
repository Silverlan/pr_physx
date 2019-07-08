#include "pr_physx/material.hpp"
#include "pr_physx/common.hpp"

pragma::physics::PxMaterial &pragma::physics::PxMaterial::GetMaterial(IMaterial &o)
{
	return *static_cast<PxMaterial*>(o.GetUserData());
}
const pragma::physics::PxMaterial &pragma::physics::PxMaterial::GetMaterial(const IMaterial &o) {return GetMaterial(const_cast<IMaterial&>(o));}
pragma::physics::PxMaterial::PxMaterial(IEnvironment &env,PxUniquePtr<physx::PxMaterial> material)
	: IMaterial{env},m_material{std::move(material)}
{
	SetUserData(this);
}
void pragma::physics::PxMaterial::Initialize()
{
	IMaterial::Initialize();
	GetInternalObject().userData = this;
}
const physx::PxMaterial &pragma::physics::PxMaterial::GetInternalObject() const {return const_cast<PxMaterial*>(this)->GetInternalObject();}
physx::PxMaterial &pragma::physics::PxMaterial::GetInternalObject() {return *m_material;}
float pragma::physics::PxMaterial::GetStaticFriction() const {return GetInternalObject().getStaticFriction();}
void pragma::physics::PxMaterial::SetStaticFriction(float friction) {GetInternalObject().setStaticFriction(friction);}
float pragma::physics::PxMaterial::GetDynamicFriction() const {return GetInternalObject().getDynamicFriction();}
void pragma::physics::PxMaterial::SetDynamicFriction(float friction) {GetInternalObject().setDynamicFriction(friction);}
float pragma::physics::PxMaterial::GetRestitution() const {return GetInternalObject().getRestitution();}
void pragma::physics::PxMaterial::SetRestitution(float restitution) {GetInternalObject().setRestitution(umath::clamp(restitution,0.f,1.f));}
