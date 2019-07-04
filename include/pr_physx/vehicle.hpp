#ifndef __PR_PX_VEHICLE_HPP__
#define __PR_PX_VEHICLE_HPP__

#include <pragma/physics/shape.hpp>
#include "pr_physx/common.hpp"

namespace pragma::physics
{
	class PxEnvironment;
	class PxVehicle
	{
	public:
		PxVehicle(const PxSharedHandle<physx::PxShape> &shape,const std::shared_ptr<physx::PxGeometry> &geometry);

		const physx::PxShape &GetPxShape() const;
		physx::PxShape &GetPxShape();

		//

		PxEnvironment &GetPxEnv() const;
	private:
		std::shared_ptr<physx::PxGeometry> m_geometry = nullptr;
		PxSharedHandle<physx::PxShape> m_shape = px_shared_null_handle<physx::PxShape>();
	};
};

#endif
