/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef __PR_PX_MATERIAL_HPP__
#define __PR_PX_MATERIAL_HPP__

#include "pr_physx/common.hpp"
#include <pragma/physics/phys_material.hpp>
#include <memory>

namespace physx
{
	class PxMaterial;
};
namespace pragma::physics
{
	class PhysXEnvironment;
	class PhysXMaterial
		: virtual public pragma::physics::IMaterial
	{
	public:
		using Index = uint16_t;
		friend PhysXEnvironment;
		friend IEnvironment;
		static PhysXMaterial &GetMaterial(IMaterial &o);
		static const PhysXMaterial &GetMaterial(const IMaterial &o);
		const physx::PxMaterial &GetInternalObject() const;
		physx::PxMaterial &GetInternalObject();

		virtual float GetStaticFriction() const override;
		virtual void SetStaticFriction(float friction) override;
		virtual float GetDynamicFriction() const override;
		virtual void SetDynamicFriction(float friction) override;
		virtual float GetRestitution() const override;
		virtual void SetRestitution(float restitution) override;
	private:
		PhysXMaterial(IEnvironment &env,PhysXUniquePtr<physx::PxMaterial> material);
		virtual void Initialize() override;
		PhysXUniquePtr<physx::PxMaterial> m_material = px_null_ptr<physx::PxMaterial>();
	};
};

#endif
