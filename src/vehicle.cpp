#include "pr_physx/vehicle.hpp"
#include "pr_physx/environment.hpp"
#include "pr_physx/collision_object.hpp"
#include "pr_physx/query_filter_callback.hpp"

enum
{
	DRIVABLE_SURFACE = 64,
	UNDRIVABLE_SURFACE = 32
};
pragma::physics::PhysXVehicle &pragma::physics::PhysXVehicle::GetVehicle(IVehicle &c)
{
	return *static_cast<PhysXVehicle*>(c.GetUserData());
}
const pragma::physics::PhysXVehicle &GetVehicle(const pragma::physics::IVehicle &v) {return GetVehicle(const_cast<pragma::physics::IVehicle&>(v));}
pragma::physics::PhysXVehicle::PhysXVehicle(IEnvironment &env,PhysXUniquePtr<physx::PxVehicleDrive> vhc,const util::TSharedHandle<ICollisionObject> &collisionObject)
	: IVehicle{env,collisionObject},m_vehicle{std::move(vhc)}
{
	SetUserData(this);
}
physx::PxVehicleDrive &pragma::physics::PhysXVehicle::GetInternalObject() const {return *m_vehicle;}
void pragma::physics::PhysXVehicle::Simulate(float dt)
{
	std::array<physx::PxVehicleWheels*,1> vehicles = {m_vehicle.get()};
	PxVehicleSuspensionRaycasts(m_raycastBatchQuery.get(),vehicles.size(),vehicles.data(),GetWheelCount(),m_raycastQueryResultPerWheel.data());

	Vector3 gravity {};
	auto *pColObj = GetCollisionObject();
	if(pColObj != nullptr)
		gravity = pColObj->GetGravity();

	gravity = {0.f,-6000.f *dt,0.f}; // TODO

	std::array<physx::PxVehicleWheelQueryResult,1> vehicleWheelQueryResults = {{m_wheelQueryResults.data(),m_wheelQueryResults.size()}};
	physx::PxVehicleUpdates(dt,GetPxEnv().ToPhysXVector(gravity),GetPxEnv().GetVehicleSurfaceTireFrictionPairs(),vehicles.size(),vehicles.data(),vehicleWheelQueryResults.data());

	
	// Check
	/*if(vehicleWheelQueryResults.front().wheelQueryResults->isInAir)
		Con::cout<<"Vehicle is in air!"<<Con::endl;
	Con::cout<<"Tire Friction: "<<vehicleWheelQueryResults.front().wheelQueryResults->tireFriction<<Con::endl;
	if(vehicleWheelQueryResults.front().wheelQueryResults->tireContactActor && vehicleWheelQueryResults.front().wheelQueryResults->tireContactActor->is<physx::PxRigidActor>())
	{
		auto *pCollisionObject = PhysXEnvironment::GetCollisionObject(*vehicleWheelQueryResults.front().wheelQueryResults->tireContactActor->is<physx::PxRigidActor>());
		if(pCollisionObject)
			Con::cout<<"HIT COLLISION OBJECT!"<<Con::endl;
	}*/
}
void pragma::physics::PhysXVehicle::Initialize()
{
	IVehicle::Initialize();
	// TODO
	// GetInternalObject().setUserData(this);

	auto numWheels = GetWheelCount();
	m_raycastQueryResultPerWheel.resize(numWheels);
	m_wheelQueryResults.resize(numWheels);
	m_raycastHitPerWheel.resize(numWheels);

	physx::PxBatchQueryDesc sqDesc {numWheels,0,0};
	sqDesc.queryMemory.userRaycastResultBuffer = m_raycastQueryResultPerWheel.data();
	sqDesc.queryMemory.userRaycastTouchBuffer = m_raycastHitPerWheel.data();
	sqDesc.queryMemory.raycastTouchBufferSize = numWheels;
	sqDesc.preFilterShader = [](
		physx::PxFilterData queryFilterData, physx::PxFilterData objectFilterData,
		const void* constantBlock, physx::PxU32 constantBlockSize,
		physx::PxHitFlags& hitFlags
	) -> physx::PxQueryHitType::Enum {
			//return (((objectFilterData.word3 & DRIVABLE_SURFACE) == 0) ?
			//	physx::PxQueryHitType::eNONE : physx::PxQueryHitType::eBLOCK);
			return (((objectFilterData.word3 & UNDRIVABLE_SURFACE) != 0) ?
				physx::PxQueryHitType::eNONE : physx::PxQueryHitType::eBLOCK);
	};
	m_raycastBatchQuery = px_create_unique_ptr(GetPxEnv().GetScene().createBatchQuery(sqDesc));


	//m_vehicle->mDriveDynData.setUseAutoGears(true);
	m_vehicle->mDriveDynData.setAnalogInput(physx::PxVehicleDrive4WControl::eANALOG_INPUT_ACCEL,1.0);
	m_vehicle->mDriveDynData.forceGearChange(1);
	//m_vehicle->mDriveDynData.setAutoBoxSwitchTime(0.f);

	//m_vehicle->mDriveDynData.setEngineRotationSpeed(100.f);
	//m_vehicle->mWheelsDynData.setWheelRotationSpeed(0,100.f);
	//vehDrive4W->mDriveDynData.
}
uint32_t pragma::physics::PhysXVehicle::GetWheelCount() const {return 4;}
float pragma::physics::PhysXVehicle::GetForwardSpeed() const
{
	return GetPxEnv().FromPhysXLength(m_vehicle->computeForwardSpeed());
}
float pragma::physics::PhysXVehicle::GetSidewaysSpeed() const
{
	return GetPxEnv().FromPhysXLength(m_vehicle->computeSidewaysSpeed());
}
void pragma::physics::PhysXVehicle::RemoveWorldObject() {}
void pragma::physics::PhysXVehicle::DoAddWorldObject() {}
pragma::physics::PhysXEnvironment &pragma::physics::PhysXVehicle::GetPxEnv() const {return static_cast<PhysXEnvironment&>(m_physEnv);}

////////////////

pragma::physics::PhysXWheel &pragma::physics::PhysXWheel::GetWheel(IWheel &w)
{
	return *static_cast<PhysXWheel*>(w.GetUserData());
}
const pragma::physics::PhysXWheel &GetVehicle(const pragma::physics::IWheel &v) {return GetVehicle(const_cast<pragma::physics::IWheel&>(v));}

pragma::physics::PhysXWheel::PhysXWheel(IEnvironment &env)
	: IWheel{env}
{
	SetUserData(this);
}
void pragma::physics::PhysXWheel::Initialize()
{
	IWheel::Initialize();
	// TODO
	//GetInternalObject().userData = this;
}
pragma::physics::PhysXEnvironment &pragma::physics::PhysXWheel::GetPxEnv() const {return static_cast<PhysXEnvironment&>(m_physEnv);}
