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
	class PxEnvironment;
	class PxMaterial
		: virtual public pragma::physics::IMaterial
	{
	public:
		using Index = uint16_t;
		friend PxEnvironment;
		friend IEnvironment;
		static PxMaterial &GetMaterial(IMaterial &o);
		static const PxMaterial &GetMaterial(const IMaterial &o);
		const physx::PxMaterial &GetInternalObject() const;
		physx::PxMaterial &GetInternalObject();

		virtual float GetStaticFriction() const override;
		virtual void SetStaticFriction(float friction) override;
		virtual float GetDynamicFriction() const override;
		virtual void SetDynamicFriction(float friction) override;
		virtual float GetRestitution() const override;
		virtual void SetRestitution(float restitution) override;
	private:
		PxMaterial(IEnvironment &env,PxUniquePtr<physx::PxMaterial> material);
		virtual void Initialize() override;
		PxUniquePtr<physx::PxMaterial> m_material = px_null_ptr<physx::PxMaterial>();
	};
};

#endif
