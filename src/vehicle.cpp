#include "pr_physx/vehicle.hpp"
#include "pr_physx/environment.hpp"

pragma::physics::PxVehicle &pragma::physics::PxVehicle::GetVehicle(IVehicle &c)
{
	return *static_cast<PxVehicle*>(c.userData);
}
pragma::physics::PxVehicle::PxVehicle(IEnvironment &env,PxUniquePtr<physx::PxVehicleDrive> vhc)
	: IVehicle{env},m_vehicle{std::move(vhc)}
{}
pragma::physics::PxEnvironment &pragma::physics::PxVehicle::GetPxEnv() const {return static_cast<PxEnvironment&>(m_physEnv);}
