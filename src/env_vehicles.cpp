#include "pr_physx/environment.hpp"
#include <vehicle/PxVehicleSDK.h>

namespace pragma::physics
{
	struct WheelCreateInfo
	{
		float mass;
		float MOI;
		float radius;
		float width;
		Vector3 centerOffset;
	};

	struct ChassisCreateInfo
	{
		float mass;
		Vector3 CMOffset;
	};
};
static void create_wheel(const pragma::physics::WheelCreateInfo &wheelCreateInfo)
{
#if 0
	// https://docs.nvidia.com/gameworks/content/gameworkslibrary/physx/guide/Manual/Vehicles.html#setupwheelssimulationdata
	//Set up the wheels.
	PxVehicleWheelData wheels[PX_MAX_NB_WHEELS];
	{
		//Set up the wheel data structures with mass, moi, radius, width.
		for(PxU32 i = 0; i < numWheels; i++)
		{
			wheels[i].mMass = wheelMass;
			wheels[i].mMOI = wheelMOI;
			wheels[i].mRadius = wheelRadius;
			wheels[i].mWidth = wheelWidth;
		}

		//Enable the handbrake for the rear wheels only.
		wheels[PxVehicleDrive4WWheelOrder::eREAR_LEFT].mMaxHandBrakeTorque=4000.0f;
		wheels[PxVehicleDrive4WWheelOrder::eREAR_RIGHT].mMaxHandBrakeTorque=4000.0f;
		//Enable steering for the front wheels only.
		wheels[PxVehicleDrive4WWheelOrder::eFRONT_LEFT].mMaxSteer=PxPi*0.3333f;
		wheels[PxVehicleDrive4WWheelOrder::eFRONT_RIGHT].mMaxSteer=PxPi*0.3333f;
	}

	//Set up the tires.
	PxVehicleTireData tires[PX_MAX_NB_WHEELS];
	{
		//Set up the tires.
		for(PxU32 i = 0; i < numWheels; i++)
		{
			tires[i].mType = TIRE_TYPE_NORMAL;
		}
	}
#endif
}

void pragma::physics::PxEnvironment::CreateWheel(const WheelCreateInfo &createInfo)
{
	physx::PxVehicleWheelData pxWheelData {};
	pxWheelData.mRadius = ToPhysXLength(createInfo.radius);
	pxWheelData.mWidth = 0.f; // TODO
	pxWheelData.mMass = createInfo.mass;
	pxWheelData.mMOI = createInfo.MOI;
	pxWheelData.mDampingRate = 0; // TODO
	pxWheelData.mMaxBrakeTorque = 0; // TODO
	pxWheelData.mMaxHandBrakeTorque = 0; // TODO
	pxWheelData.mMaxSteer = 0; // TODO
	pxWheelData.mToeAngle = 0; // TODO

	physx::PxVehicleTireData tireData {};
	tireData.mCamberStiffnessPerUnitGravity;
	tireData.mFrictionVsSlipGraph;
	tireData.mLatStiffX;
	tireData.mLatStiffY;
	tireData.mLongitudinalStiffnessPerUnitGravity;
	tireData.mType;
}

static void create_vehicle()
{

}

void pragma::physics::PxEnvironment::CreateVehicle()
{
#if 0
	uint32_t numWheels = 0;

	physx::PxVehicleWheelsSimData* wheelsSimData = physx::PxVehicleWheelsSimData::allocate(numWheels);
	physx::setupWheelsSimulationData(wheelsSimData);

	physx::PxVehicleDriveSimData4W driveSimData;
	physx::setupDriveSimData(driveSimData);

	physx::PxRigidDynamic* vehActor = px_create_unique_ptr(m_physics->createRigidDynamic(startPose);
	setupVehicleActor(vehActor);
	m_scene->addActor(*vehActor);

	physx::PxVehicleDrive4W* vehDrive4W = physx::PxVehicleDrive4W::allocate(numWheels);
	vehDrive4W->setup(m_physics.get(), veh4WActor, *wheelsSimData, driveSimData, numWheels - 4);
	wheelsSimData->free();
#endif
}
