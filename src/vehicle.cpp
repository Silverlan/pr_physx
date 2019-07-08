#include "pr_physx/vehicle.hpp"
#include "pr_physx/environment.hpp"

pragma::physics::PxVehicle &pragma::physics::PxVehicle::GetVehicle(IVehicle &c)
{
	return *static_cast<PxVehicle*>(c.GetUserData());
}
const pragma::physics::PxVehicle &GetVehicle(const pragma::physics::IVehicle &v) {return GetVehicle(const_cast<pragma::physics::IVehicle&>(v));}
pragma::physics::PxVehicle::PxVehicle(IEnvironment &env,PxUniquePtr<physx::PxVehicleDrive> vhc)
	: IVehicle{env},m_vehicle{std::move(vhc)}
{
	SetUserData(this);
}
void pragma::physics::PxVehicle::Initialize()
{
	IVehicle::Initialize();
	// TODO
	//GetInternalObject().userData = this;
}
void pragma::physics::PxVehicle::RemoveWorldObject() {}
void pragma::physics::PxVehicle::DoAddWorldObject() {}
pragma::physics::PxEnvironment &pragma::physics::PxVehicle::GetPxEnv() const {return static_cast<PxEnvironment&>(m_physEnv);}

////////////////

pragma::physics::PxWheel &pragma::physics::PxWheel::GetWheel(IWheel &w)
{
	return *static_cast<PxWheel*>(w.GetUserData());
}
const pragma::physics::PxWheel &GetVehicle(const pragma::physics::IWheel &v) {return GetVehicle(const_cast<pragma::physics::IWheel&>(v));}

pragma::physics::PxWheel::PxWheel(IEnvironment &env)
	: IWheel{env}
{
	SetUserData(this);
}
void pragma::physics::PxWheel::Initialize()
{
	IWheel::Initialize();
	// TODO
	//GetInternalObject().userData = this;
}
pragma::physics::PxEnvironment &pragma::physics::PxWheel::GetPxEnv() const {return static_cast<PxEnvironment&>(m_physEnv);}
