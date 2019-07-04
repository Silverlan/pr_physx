#include "pr_physx/material.hpp"
#include "pr_physx/common.hpp"

pragma::physics::PxMaterial::PxMaterial(IEnvironment &env,PxUniquePtr<physx::PxMaterial> material)
	: IMaterial{env},m_material{std::move(material)}
{}
const physx::PxMaterial &pragma::physics::PxMaterial::GetPxMaterial() const {return const_cast<PxMaterial*>(this)->GetPxMaterial();}
physx::PxMaterial &pragma::physics::PxMaterial::GetPxMaterial() {return *m_material;}
float pragma::physics::PxMaterial::GetStaticFriction() const {return GetPxMaterial().getStaticFriction();}
void pragma::physics::PxMaterial::SetStaticFriction(float friction) {GetPxMaterial().setStaticFriction(friction);}
float pragma::physics::PxMaterial::GetDynamicFriction() const {return GetPxMaterial().getDynamicFriction();}
void pragma::physics::PxMaterial::SetDynamicFriction(float friction) {GetPxMaterial().setDynamicFriction(friction);}
float pragma::physics::PxMaterial::GetRestitution() const {return GetPxMaterial().getRestitution();}
void pragma::physics::PxMaterial::SetRestitution(float restitution) {GetPxMaterial().setRestitution(restitution);}
