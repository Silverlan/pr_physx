#ifndef __PR_PX_VEHICLE_HPP__
#define __PR_PX_VEHICLE_HPP__

#include <pragma/physics/vehicle.hpp>
#include "pr_physx/common.hpp"

namespace pragma::physics
{
	class PxEnvironment;
	class PxVehicleDrive;
	class PxVehicle
		: public IVehicle
	{
	public:
		static PxVehicle &GetVehicle(IVehicle &v);
		static const PxVehicle &GetVehicle(const IVehicle &v);
	private:
		PxVehicle(IEnvironment &env,PxUniquePtr<physx::PxVehicleDrive> vhc);
		PxEnvironment &GetPxEnv() const;
		PxUniquePtr<physx::PxVehicleDrive> m_vehicle = px_null_ptr<physx::PxVehicleDrive>();
	};
};

#endif
