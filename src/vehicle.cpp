#include "pr_physx/vehicle.hpp"
#include "pr_physx/environment.hpp"
#include "pr_physx/collision_object.hpp"
#include "pr_physx/query_filter_callback.hpp"
#include <vehicle/PxVehicleUtil.h>

pragma::physics::PhysXVehicle &pragma::physics::PhysXVehicle::GetVehicle(IVehicle &c)
{
	return *static_cast<PhysXVehicle*>(c.GetUserData());
}
const pragma::physics::PhysXVehicle &GetVehicle(const pragma::physics::IVehicle &v) {return GetVehicle(const_cast<pragma::physics::IVehicle&>(v));}
pragma::physics::PhysXVehicle::PhysXVehicle(
	IEnvironment &env,PhysXUniquePtr<physx::PxVehicleDrive> vhc,const util::TSharedHandle<ICollisionObject> &collisionObject,
	PhysXUniquePtr<physx::PxBatchQuery> raycastBatchQuery
)
	: IVehicle{env,collisionObject},m_vehicle{std::move(vhc)},m_raycastBatchQuery{std::move(raycastBatchQuery)}
{
	SetUserData(this);
}
physx::PxVehicleDrive &pragma::physics::PhysXVehicle::GetInternalObject() const {return *m_vehicle;}

namespace snippetvehicle
{

	using namespace physx;

	void setupDrivableSurface(PxFilterData& filterData);

	void setupNonDrivableSurface(PxFilterData& filterData);


} // namespace snippetvehicle

physx::PxF32					gVehicleModeLifetime = 4.0f;
physx::PxF32					gVehicleModeTimer = 0.0f;
physx::PxU32					gVehicleOrderProgress = 0;
bool					gVehicleOrderComplete = false;
extern snippetvehicle::VehicleSceneQueryData	gVehicleSceneQueryData;


// Source: PhysX vehicle demo samples
static physx::PxVehicleKeySmoothingData gKeySmoothingData =
{
	{
		6.0f,	//rise rate eANALOG_INPUT_ACCEL
		6.0f,	//rise rate eANALOG_INPUT_BRAKE		
		6.0f,	//rise rate eANALOG_INPUT_HANDBRAKE	
		2.5f,	//rise rate eANALOG_INPUT_STEER_LEFT
		2.5f,	//rise rate eANALOG_INPUT_STEER_RIGHT
	},
	{
		10.0f,	//fall rate eANALOG_INPUT_ACCEL
		10.0f,	//fall rate eANALOG_INPUT_BRAKE		
		10.0f,	//fall rate eANALOG_INPUT_HANDBRAKE	
		5.0f,	//fall rate eANALOG_INPUT_STEER_LEFT
		5.0f	//fall rate eANALOG_INPUT_STEER_RIGHT
	}
};

static physx::PxVehiclePadSmoothingData gPadSmoothingData =
{
	{
		6.0f,		//rise rate eANALOG_INPUT_ACCEL
		6.0f,		//rise rate eANALOG_INPUT_BRAKE		
		6.0f,		//rise rate eANALOG_INPUT_HANDBRAKE	
		2.5f,		//rise rate eANALOG_INPUT_STEER_LEFT
		2.5f,		//rise rate eANALOG_INPUT_STEER_RIGHT
	},
	{
		10.0f,		//fall rate eANALOG_INPUT_ACCEL
		10.0f,		//fall rate eANALOG_INPUT_BRAKE		
		10.0f,		//fall rate eANALOG_INPUT_HANDBRAKE	
		5.0f,		//fall rate eANALOG_INPUT_STEER_LEFT
		5.0f		//fall rate eANALOG_INPUT_STEER_RIGHT
	}
};
extern physx::PxFixedSizeLookupTable<8> gSteerVsForwardSpeedTable;
void pragma::physics::PhysXVehicle::Simulate(float dt)
{
	if(IsSpawned() == false)
		return;
	auto *gVehicle4W = static_cast<physx::PxVehicleDrive4W*>(m_vehicle.get());

	//Update the control inputs for the vehicle.
	if(ShouldUseDigitalInputs())
		physx::PxVehicleDrive4WSmoothDigitalRawInputsAndSetAnalogInputs(gKeySmoothingData, gSteerVsForwardSpeedTable, m_inputData, dt, IsInAir(), *gVehicle4W);
	else
		physx::PxVehicleDrive4WSmoothAnalogRawInputsAndSetAnalogInputs(gPadSmoothingData, gSteerVsForwardSpeedTable, m_inputData, dt, IsInAir(), *gVehicle4W);

	//Raycasts.
	physx::PxVehicleWheels* vehicles[1] = {gVehicle4W};
	physx::PxRaycastQueryResult* raycastResults = gVehicleSceneQueryData.getRaycastQueryResultBuffer(0);
	const physx::PxU32 raycastResultsSize = gVehicleSceneQueryData.getQueryResultBufferSize();
	physx::PxVehicleSuspensionRaycasts(m_raycastBatchQuery.get(), 1, vehicles, raycastResultsSize, raycastResults);

	//Vehicle update.
	const physx::PxVec3 grav = GetPxEnv().GetScene().getGravity();//ToPhysXVector(Vector3{0.f,-9.81f *40.f,0.f});//gScene->getGravity();
	physx::PxWheelQueryResult wheelQueryResults[PX_MAX_NB_WHEELS];
	physx::PxVehicleWheelQueryResult vehicleQueryResults[1] = {{wheelQueryResults, gVehicle4W->mWheelsSimData.getNbWheels()}};
	
	PxVehicleUpdates(dt, grav,GetPxEnv().GetVehicleSurfaceTireFrictionPairs(), 1, vehicles, vehicleQueryResults);

	//Work out if the vehicle is in the air.
	umath::set_flag(m_stateFlags,StateFlags::InAir,gVehicle4W->getRigidDynamicActor()->isSleeping() ? false : PxVehicleIsInAir(vehicleQueryResults[0]));
}
void pragma::physics::PhysXVehicle::InitializeSceneBatchQuery(const snippetvehicle::VehicleSceneQueryData& vehicleSceneQueryData)
{
	const physx::PxU32 maxNumQueriesInBatch =  vehicleSceneQueryData.mNumQueriesPerBatch;
	const physx::PxU32 maxNumHitResultsInBatch = vehicleSceneQueryData.mNumQueriesPerBatch*vehicleSceneQueryData.mNumHitResultsPerQuery;

	physx::PxBatchQueryDesc sqDesc(maxNumQueriesInBatch, maxNumQueriesInBatch, 0);

	sqDesc.queryMemory.userRaycastResultBuffer = const_cast<physx::PxRaycastQueryResult*>(&vehicleSceneQueryData.mRaycastResults.at(maxNumQueriesInBatch));
	sqDesc.queryMemory.userRaycastTouchBuffer = const_cast<physx::PxRaycastHit*>(&vehicleSceneQueryData.mRaycastHitBuffer.at(maxNumHitResultsInBatch));
	sqDesc.queryMemory.raycastTouchBufferSize = maxNumHitResultsInBatch;

	sqDesc.queryMemory.userSweepResultBuffer = const_cast<physx::PxSweepQueryResult*>(&vehicleSceneQueryData.mSweepResults.at(maxNumQueriesInBatch));
	sqDesc.queryMemory.userSweepTouchBuffer = const_cast<physx::PxSweepHit*>(&vehicleSceneQueryData.mSweepHitBuffer.at(maxNumHitResultsInBatch));
	sqDesc.queryMemory.sweepTouchBufferSize = maxNumHitResultsInBatch;

	sqDesc.preFilterShader = vehicleSceneQueryData.mPreFilterShader;

	sqDesc.postFilterShader = vehicleSceneQueryData.mPostFilterShader;

	m_raycastBatchQuery = px_create_unique_ptr(GetPxEnv().GetScene().createBatchQuery(sqDesc));
}
void pragma::physics::PhysXVehicle::Initialize()
{
	IVehicle::Initialize();
	// TODO
	// GetInternalObject().setUserData(this);
	//InitializeSceneBatchQuery();

#if 0
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
#endif


	//m_vehicle->mDriveDynData.setUseAutoGears(true);
	//m_vehicle->mDriveDynData.setAnalogInput(physx::PxVehicleDrive4WControl::eANALOG_INPUT_ACCEL,1.0);
//m_vehicle->mDriveDynData.forceGearChange(1);
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
void pragma::physics::PhysXVehicle::RemoveWorldObject()
{
	if(m_collisionObject == nullptr)
		return;
	m_collisionObject->RemoveWorldObject();
}
void pragma::physics::PhysXVehicle::DoAddWorldObject() {}
void pragma::physics::PhysXVehicle::DoSpawn()
{
	IVehicle::DoSpawn();
	if(m_collisionObject == nullptr)
		return;
	m_collisionObject->Spawn();
}
pragma::physics::PhysXEnvironment &pragma::physics::PhysXVehicle::GetPxEnv() const {return static_cast<PhysXEnvironment&>(m_physEnv);}

void pragma::physics::PhysXVehicle::SetUseDigitalInputs(bool bUseDigitalInputs) {umath::set_flag(m_stateFlags,StateFlags::UseDigitalInputs,bUseDigitalInputs);}
bool pragma::physics::PhysXVehicle::ShouldUseDigitalInputs() const {return umath::is_flag_set(m_stateFlags,StateFlags::UseDigitalInputs);}

void pragma::physics::PhysXVehicle::SetBrakeFactor(float f)
{
	if(ShouldUseDigitalInputs())
		m_inputData.setDigitalBrake(AnalogInputToDigital(f));
	else
		m_inputData.setAnalogBrake(f);
}
void pragma::physics::PhysXVehicle::SetHandbrakeFactor(float f)
{
	if(ShouldUseDigitalInputs())
		m_inputData.setDigitalHandbrake(AnalogInputToDigital(f));
	else
		m_inputData.setAnalogHandbrake(f);
}
void pragma::physics::PhysXVehicle::SetAccelerationFactor(float f)
{
	if(ShouldUseDigitalInputs())
		m_inputData.setDigitalAccel(AnalogInputToDigital(f));
	else
		m_inputData.setAnalogAccel(f);
}
void pragma::physics::PhysXVehicle::SetTurnFactor(float f)
{
	if(ShouldUseDigitalInputs())
	{
		if(f < 0.f)
			m_inputData.setDigitalSteerLeft(AnalogInputToDigital(-f));
		else
			m_inputData.setDigitalSteerRight(AnalogInputToDigital(f));
	}
	else
		m_inputData.setAnalogSteer(f);
}

void pragma::physics::PhysXVehicle::ResetControls()
{
	if(ShouldUseDigitalInputs())
	{
		m_inputData.setDigitalAccel(false);
		m_inputData.setDigitalSteerLeft(false);
		m_inputData.setDigitalSteerRight(false);
		m_inputData.setDigitalBrake(false);
		m_inputData.setDigitalHandbrake(false);
	}
	else
	{
		m_inputData.setAnalogAccel(0.0f);
		m_inputData.setAnalogSteer(0.0f);
		m_inputData.setAnalogBrake(0.0f);
		m_inputData.setAnalogHandbrake(0.0f);
	}
}

void pragma::physics::PhysXVehicle::SetGear(Gear gear)
{
	m_vehicle->mDriveDynData.forceGearChange(ToPhysXGear(gear));
}
void pragma::physics::PhysXVehicle::SetGearDown()
{
	m_inputData.setGearDown(true);
}
void pragma::physics::PhysXVehicle::SetGearUp()
{
	m_inputData.setGearUp(true);
}
void pragma::physics::PhysXVehicle::SetGearSwitchTime(float time)
{
	m_vehicle->mDriveDynData.setGearSwitchTime(time);
}
void pragma::physics::PhysXVehicle::ChangeToGear(Gear gear)
{
	m_vehicle->mDriveDynData.setTargetGear(ToPhysXGear(gear));
}
void pragma::physics::PhysXVehicle::SetUseAutoGears(bool useAutoGears)
{
	m_vehicle->mDriveDynData.setUseAutoGears(useAutoGears);
}

bool pragma::physics::PhysXVehicle::ShouldUseAutoGears() const
{
	return m_vehicle->mDriveDynData.getUseAutoGears();
}
pragma::physics::PhysXVehicle::Gear pragma::physics::PhysXVehicle::GetCurrentGear() const
{
	return FromPhysXGear(m_vehicle->mDriveDynData.getCurrentGear());
}
float pragma::physics::PhysXVehicle::GetEngineRotationSpeed() const
{
	return m_vehicle->mDriveDynData.getEngineRotationSpeed();
}

void pragma::physics::PhysXVehicle::SetRestState()
{
	m_vehicle->mDriveDynData.setToRestState();
}

void pragma::physics::PhysXVehicle::SetWheelRotationAngle(WheelIndex wheel,umath::Radian angle)
{
	m_vehicle->mWheelsDynData.setWheelRotationAngle(wheel,angle);
}
void pragma::physics::PhysXVehicle::SetWheelRotationSpeed(WheelIndex wheel,umath::Radian speed)
{
	m_vehicle->mWheelsDynData.setWheelRotationSpeed(wheel,speed);
}

bool pragma::physics::PhysXVehicle::IsInAir() const {return umath::is_flag_set(m_stateFlags,StateFlags::InAir);}

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
