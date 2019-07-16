#include "pr_physx/environment.hpp"
#include "pr_physx/vehicle.hpp"
#include "pr_physx/material.hpp"
#include "pr_physx/collision_object.hpp"
#include "pr_physx/shape.hpp"
#include <pragma/networkstate/networkstate.h>
#include <vehicle/PxVehicleSDK.h>

#pragma optimize("",off)
physx::PxF32 gLengthScale = 1.f /0.025f;
namespace snippetvehicle
{

	using namespace physx;

	void setupDrivableSurface(PxFilterData& filterData)
	{
		filterData.word3 = static_cast<PxU32>(DRIVABLE_SURFACE);
	}

	void setupNonDrivableSurface(PxFilterData& filterData)
	{
		filterData.word3 = UNDRIVABLE_SURFACE;
	}

	PxQueryHitType::Enum WheelSceneQueryPreFilterBlocking
	(PxFilterData filterData0, PxFilterData filterData1,
		const void* constantBlock, PxU32 constantBlockSize,
		PxHitFlags& queryFlags)
	{
		//filterData0 is the vehicle suspension query.
		//filterData1 is the shape potentially hit by the query.
		PX_UNUSED(filterData0);
		PX_UNUSED(constantBlock);
		PX_UNUSED(constantBlockSize);
		PX_UNUSED(queryFlags);
		return ((0 == (filterData1.word3 & DRIVABLE_SURFACE)) ? PxQueryHitType::eNONE : PxQueryHitType::eBLOCK);
	}

	VehicleSceneQueryData VehicleSceneQueryData::allocate
	(const PxU32 maxNumVehicles, const PxU32 maxNumWheelsPerVehicle, const PxU32 maxNumHitPointsPerWheel, const PxU32 numVehiclesInBatch, 
		PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader)
	{
		const PxU32 maxNumWheels = maxNumVehicles*maxNumWheelsPerVehicle;
		const PxU32 maxNumHitPoints = maxNumWheels*maxNumHitPointsPerWheel;
		VehicleSceneQueryData sqData {};
		sqData.mNumQueriesPerBatch = numVehiclesInBatch*maxNumWheelsPerVehicle;
		sqData.mNumHitResultsPerQuery = maxNumHitPointsPerWheel;

		sqData.mRaycastResults.resize(maxNumWheels);

		sqData.mRaycastHitBuffer.resize(maxNumHitPoints);

		sqData.mSweepResults.resize(maxNumWheels);

		sqData.mSweepHitBuffer.resize(maxNumHitPoints);

		sqData.mPreFilterShader = preFilterShader;
		sqData.mPostFilterShader = postFilterShader;

		return sqData;
	}

	PxBatchQuery* VehicleSceneQueryData::setUpBatchedSceneQuery(const PxU32 batchId, const VehicleSceneQueryData& vehicleSceneQueryData, PxScene* scene)
	{
		const PxU32 maxNumQueriesInBatch =  vehicleSceneQueryData.mNumQueriesPerBatch;
		const PxU32 maxNumHitResultsInBatch = vehicleSceneQueryData.mNumQueriesPerBatch*vehicleSceneQueryData.mNumHitResultsPerQuery;

		PxBatchQueryDesc sqDesc(maxNumQueriesInBatch, maxNumQueriesInBatch, 0);

		sqDesc.queryMemory.userRaycastResultBuffer = const_cast<physx::PxRaycastQueryResult*>(&vehicleSceneQueryData.mRaycastResults.at(batchId*maxNumQueriesInBatch));
		sqDesc.queryMemory.userRaycastTouchBuffer = const_cast<physx::PxRaycastHit*>(&vehicleSceneQueryData.mRaycastHitBuffer.at(batchId*maxNumHitResultsInBatch));
		sqDesc.queryMemory.raycastTouchBufferSize = maxNumHitResultsInBatch;

		sqDesc.queryMemory.userSweepResultBuffer = const_cast<physx::PxSweepQueryResult*>(&vehicleSceneQueryData.mSweepResults.at(batchId*maxNumQueriesInBatch));
		sqDesc.queryMemory.userSweepTouchBuffer = const_cast<physx::PxSweepHit*>(&vehicleSceneQueryData.mSweepHitBuffer.at(batchId*maxNumHitResultsInBatch));
		sqDesc.queryMemory.sweepTouchBufferSize = maxNumHitResultsInBatch;

		sqDesc.preFilterShader = vehicleSceneQueryData.mPreFilterShader;

		sqDesc.postFilterShader = vehicleSceneQueryData.mPostFilterShader;

		return scene->createBatchQuery(sqDesc);
	}

	PxRaycastQueryResult* VehicleSceneQueryData::getRaycastQueryResultBuffer(const PxU32 batchId) 
	{
		return const_cast<physx::PxRaycastQueryResult*>(&mRaycastResults.at(batchId*mNumQueriesPerBatch));
	}

	PxSweepQueryResult* VehicleSceneQueryData::getSweepQueryResultBuffer(const PxU32 batchId)
	{
		return &(mSweepResults.at(batchId*mNumQueriesPerBatch));
	}


	PxU32 VehicleSceneQueryData::getQueryResultBufferSize() const 
	{
		return mNumQueriesPerBatch;
	}

} // namespace snippetvehicle
enum
{
	TIRE_TYPE_NORMAL=0,
	TIRE_TYPE_WETS,
	TIRE_TYPE_SLICKS,
	TIRE_TYPE_ICE,
	TIRE_TYPE_MUD,
	MAX_NUM_TIRE_TYPES
};

namespace pragma::physics
{
	struct TireCreateInfo
	{
		enum class Type : uint32_t
		{
			Normal = 0,
			Slicks,
			Wets,
			Snow,
			Winter,
			Summer,
			AllTerrain,
			Mud
		};
		Vector2 lateralStiffness;
		float longitudinalStiffnessPerUnitGravity;
		float camberStiffnessPerUnitGravity;
		std::array<std::array<float,2>,3> frictionVsSlipGraph;
		Type type = Type::Normal;
	};
};
void customizeVehicleToLengthScale(const physx::PxReal lengthScale, physx::PxRigidDynamic* rigidDynamic, physx::PxVehicleWheelsSimData* wheelsSimData, physx::PxVehicleDriveSimData* driveSimData)
{
	//Rigid body center of mass and moment of inertia.
	{
		physx::PxTransform t = rigidDynamic->getCMassLocalPose();
		t.p *= lengthScale;
		rigidDynamic->setCMassLocalPose(t);

		physx::PxVec3 moi = rigidDynamic->getMassSpaceInertiaTensor();
		moi *= (lengthScale*lengthScale);
		rigidDynamic->setMassSpaceInertiaTensor(moi);
	}

	//Wheels, suspensions, wheel centers, tire/susp force application points.
	{
		for(physx::PxU32 i = 0; i < wheelsSimData->getNbWheels(); i++)
		{
			physx::PxVehicleWheelData wheelData = wheelsSimData->getWheelData(i);
			wheelData.mRadius *= lengthScale;
			wheelData.mWidth *= lengthScale;
			wheelData.mDampingRate *= lengthScale*lengthScale;
			wheelData.mMaxBrakeTorque *= lengthScale*lengthScale;
			wheelData.mMaxHandBrakeTorque *= lengthScale*lengthScale;
			wheelData.mMOI *= lengthScale*lengthScale;
			wheelsSimData->setWheelData(i, wheelData);

			physx::PxVehicleSuspensionData suspData = wheelsSimData->getSuspensionData(i);
			suspData.mMaxCompression *= lengthScale;
			suspData.mMaxDroop *= lengthScale;
			wheelsSimData->setSuspensionData(i, suspData);

			physx::PxVec3 v = wheelsSimData->getWheelCentreOffset(i);
			v *= lengthScale;
			wheelsSimData->setWheelCentreOffset(i,v);

			v = wheelsSimData->getSuspForceAppPointOffset(i);
			v *= lengthScale;
			wheelsSimData->setSuspForceAppPointOffset(i,v);

			v = wheelsSimData->getTireForceAppPointOffset(i);
			v *= lengthScale;
			wheelsSimData->setTireForceAppPointOffset(i,v);
		}
	}

	//Slow forward speed correction.
	{
		wheelsSimData->setSubStepCount(5.0f*lengthScale, 3, 1);
		wheelsSimData->setMinLongSlipDenominator(4.0f*lengthScale);
	}

	//Engine
	if(driveSimData)
	{
		physx::PxVehicleEngineData engineData = driveSimData->getEngineData();
		engineData.mMOI *= lengthScale*lengthScale;
		engineData.mPeakTorque *= lengthScale*lengthScale;
		engineData.mDampingRateFullThrottle *= lengthScale*lengthScale;
		engineData.mDampingRateZeroThrottleClutchEngaged *= lengthScale*lengthScale;
		engineData.mDampingRateZeroThrottleClutchDisengaged *= lengthScale*lengthScale;
		driveSimData->setEngineData(engineData);
	}

	//Clutch.
	if(driveSimData)
	{
		physx::PxVehicleClutchData clutchData = driveSimData->getClutchData();
		clutchData.mStrength *= lengthScale*lengthScale;
		driveSimData->setClutchData(clutchData);
	}	
}

void customizeVehicleToLengthScale(const physx::PxReal lengthScale, physx::PxRigidDynamic* rigidDynamic, physx::PxVehicleWheelsSimData* wheelsSimData, physx::PxVehicleDriveSimData4W* driveSimData)
{
	customizeVehicleToLengthScale(lengthScale, rigidDynamic, wheelsSimData, static_cast<physx::PxVehicleDriveSimData*>(driveSimData));

	//Ackermann geometry.
	if(driveSimData)
	{
		physx::PxVehicleAckermannGeometryData ackermannData = driveSimData->getAckermannGeometryData();
		ackermannData.mAxleSeparation *= lengthScale;
		ackermannData.mFrontWidth *= lengthScale;
		ackermannData.mRearWidth *= lengthScale;
		driveSimData->setAckermannGeometryData(ackermannData);
	}
}

static void setupNonDrivableSurface(physx::PxFilterData& filterData)
{
	filterData.word3 = UNDRIVABLE_SURFACE;
}

util::TSharedHandle<pragma::physics::PhysXRigidBody> createVehicleActor
(const physx::PxVehicleChassisData& chassisData,
	const pragma::physics::VehicleCreateInfo &vhcDesc, const physx::PxFilterData& wheelSimFilterData,
	const physx::PxU32 numChassisMeshes, const physx::PxFilterData& chassisSimFilterData,
	physx::PxPhysics& physics)
{
	//We need a rigid body actor for the vehicle.
	//Don't forget to add the actor to the scene after setting up the associated vehicle.

	//Add the chassis shapes to the actor.


	//Wheel and chassis query filter data.
	//Optional: cars don't drive on other cars.
	physx::PxFilterData wheelQryFilterData;
	setupNonDrivableSurface(wheelQryFilterData);
	physx::PxFilterData chassisQryFilterData;
	setupNonDrivableSurface(chassisQryFilterData);

	auto &rigidBody = pragma::physics::PhysXCollisionObject::GetCollisionObject(*vhcDesc.actor);

	auto &actorShapes = rigidBody.GetActorShapeCollection().GetActorShapes();
	if(vhcDesc.chassis.shapeIndex >= 0 && vhcDesc.chassis.shapeIndex < actorShapes.size())
	{
		auto &chassisActorShape = actorShapes.at(vhcDesc.chassis.shapeIndex);
		auto &pxActorShape = chassisActorShape->GetActorShape();
		pxActorShape.setQueryFilterData(chassisQryFilterData);
		pxActorShape.setSimulationFilterData(chassisSimFilterData);
		pxActorShape.setLocalPose(physx::PxTransform(physx::PxIdentity));
	}
	for(auto &wheelDesc : vhcDesc.wheels)
	{
		if(wheelDesc.shapeIndex < 0 || wheelDesc.shapeIndex >= actorShapes.size())
			continue;
		auto &pxActorShape = actorShapes.at(wheelDesc.shapeIndex)->GetActorShape();
		pxActorShape.setQueryFilterData(wheelQryFilterData);
		pxActorShape.setSimulationFilterData(wheelSimFilterData);
		pxActorShape.setLocalPose(physx::PxTransform(physx::PxIdentity));
	}

	auto &dynamicRigidActor = static_cast<physx::PxRigidDynamic&>(rigidBody.GetInternalObject());
	dynamicRigidActor.setMass(chassisData.mMass);
	dynamicRigidActor.setMassSpaceInertiaTensor(chassisData.mMOI);
	dynamicRigidActor.setCMassLocalPose(physx::PxTransform(chassisData.mCMOffset,physx::PxQuat(physx::PxIdentity)));
	return util::shared_handle_cast<pragma::physics::IBase,pragma::physics::PhysXRigidBody>(rigidBody.ClaimOwnership());
}

void setupWheelsSimulationData
(pragma::physics::PhysXEnvironment &env,const pragma::physics::VehicleCreateInfo &vhcCreateInfo,
	const physx::PxVec3& chassisCMOffset, const physx::PxF32 chassisMass,
	physx::PxVehicleWheelsSimData* wheelsSimData)
{
	//Set up the wheels.
	struct WheelData
	{
		physx::PxVehicleWheelData data;
		physx::PxVehicleTireData tire;
		physx::PxVehicleSuspensionData suspension;

		physx::PxVec3 suspTravelDirection;
		physx::PxVec3 wheelCentreCMOffset;
		physx::PxVec3 suspForceAppCMOffset;
		physx::PxVec3 tireForceAppCMOffset;
	};
	std::vector<WheelData> wheels {};
	wheels.reserve(vhcCreateInfo.wheels.size());
	std::vector<physx::PxVec3> wheelCenterActorOffsets;
	for(auto &createInfo : vhcCreateInfo.wheels)
	{
		wheels.push_back({});
		auto &wheel = wheels.back();

		wheel.data.mMass = createInfo.mass;
		wheel.data.mRadius = createInfo.radius /gLengthScale;
		wheel.data.mWidth = createInfo.width /gLengthScale;
		wheel.data.mMaxHandBrakeTorque = createInfo.maxHandbrakeTorque /umath::pow2(gLengthScale);
		wheel.data.mMaxSteer = umath::deg_to_rad(createInfo.maxSteeringAngle);
		wheel.data.mMOI = createInfo.GetMomentOfInertia() /umath::pow2(gLengthScale);

		wheel.tire.mType = TIRE_TYPE_NORMAL;

		wheelCenterActorOffsets.push_back(env.ToPhysXVector(createInfo.chassisOffset) /gLengthScale);
	}

	std::vector<physx::PxF32> suspSprungMasses(vhcCreateInfo.wheels.size());
	physx::PxVehicleComputeSprungMasses(
		vhcCreateInfo.wheels.size(),wheelCenterActorOffsets.data(), 
		chassisCMOffset,chassisMass,1,suspSprungMasses.data()
	); // TODO :Gravity direction???

	uint32_t wheelIdx = 0u;
	for(auto &wheel : wheels)
	{
		auto &wheelCreateInfo = vhcCreateInfo.wheels.at(wheelIdx);
		auto &suspension = wheel.suspension;
		suspension.mMaxCompression = wheelCreateInfo.suspension.maxCompression;
		suspension.mMaxDroop = wheelCreateInfo.suspension.maxDroop;
		suspension.mSpringStrength = wheelCreateInfo.suspension.springStrength;	
		suspension.mSpringDamperRate = wheelCreateInfo.suspension.springDamperRate;
		suspension.mSprungMass = suspSprungMasses.at(wheelIdx);

		auto leftWheel = umath::is_flag_set(wheelCreateInfo.flags,pragma::physics::WheelCreateInfo::Flags::Right) == false;
		auto factor = leftWheel ? 1.f : -1.f;
		suspension.mCamberAtRest = wheelCreateInfo.suspension.camberAngleAtRest;
		suspension.mCamberAtMaxDroop = wheelCreateInfo.suspension.camberAngleAtMaxDroop;
		suspension.mCamberAtMaxCompression = wheelCreateInfo.suspension.camberAngleAtMaxCompression;

		//Vertical suspension travel.
		wheel.suspTravelDirection = physx::PxVec3(0,-1,0);

		//Wheel center offset is offset from rigid body center of mass.
		wheel.wheelCentreCMOffset = wheelCenterActorOffsets.at(wheelIdx) -chassisCMOffset;

		//Suspension force application point 0.3 metres below 
		//rigid body center of mass.
		wheel.suspForceAppCMOffset = physx::PxVec3(wheel.wheelCentreCMOffset.x,-0.3f,wheel.wheelCentreCMOffset.z);

		//Tire force application point 0.3 metres below 
		//rigid body center of mass.
		wheel.tireForceAppCMOffset = physx::PxVec3(wheel.wheelCentreCMOffset.x,-0.3f,wheel.wheelCentreCMOffset.z);

		++wheelIdx;
	}

	//Set up the filter data of the raycast that will be issued by each suspension.
	physx::PxFilterData qryFilterData;
	setupNonDrivableSurface(qryFilterData);

	wheelIdx = 0u;
	for(auto &wheel : wheels)
	{
		auto &wheelCreateInfo = vhcCreateInfo.wheels.at(wheelIdx);
		wheelsSimData->setWheelData(wheelIdx,wheel.data);
		wheelsSimData->setTireData(wheelIdx,wheel.tire);
		wheelsSimData->setSuspensionData(wheelIdx,wheel.suspension);
		wheelsSimData->setSuspTravelDirection(wheelIdx,wheel.suspTravelDirection);
		wheelsSimData->setWheelCentreOffset(wheelIdx,wheel.wheelCentreCMOffset);
		wheelsSimData->setSuspForceAppPointOffset(wheelIdx,wheel.suspForceAppCMOffset);
		wheelsSimData->setTireForceAppPointOffset(wheelIdx,wheel.tireForceAppCMOffset);
		wheelsSimData->setSceneQueryFilterData(wheelIdx,qryFilterData);
		wheelsSimData->setWheelShapeMapping(wheelIdx,wheelCreateInfo.shapeIndex);
		++wheelIdx;
	}

	const auto getWheelEnum = [](pragma::physics::VehicleCreateInfo::Wheel wheel) {
		switch(wheel)
		{
		case pragma::physics::VehicleCreateInfo::Wheel::FrontLeft:
			return physx::PxVehicleDrive4WWheelOrder::eFRONT_LEFT;
		case pragma::physics::VehicleCreateInfo::Wheel::FrontRight:
			return physx::PxVehicleDrive4WWheelOrder::eFRONT_RIGHT;
		case pragma::physics::VehicleCreateInfo::Wheel::RearLeft:
			return physx::PxVehicleDrive4WWheelOrder::eREAR_LEFT;
		case pragma::physics::VehicleCreateInfo::Wheel::RearRight:
			return physx::PxVehicleDrive4WWheelOrder::eREAR_RIGHT;
		}
	};
	for(auto &antiRollBarDesc : vhcCreateInfo.antiRollBars)
	{
		physx::PxVehicleAntiRollBarData bar {};
		bar.mWheel0 = getWheelEnum(antiRollBarDesc.wheel0);
		bar.mWheel1 = getWheelEnum(antiRollBarDesc.wheel1);
		bar.mStiffness = antiRollBarDesc.stiffness;
		wheelsSimData->addAntiRollBarData(bar);
	}
}

struct ShapeUserData
{
	ShapeUserData()
		: isWheel(false),
		wheelId(0xffffffff)
	{
	}

	bool isWheel;
	physx::PxU32 wheelId;
};

struct VehicleDesc
{
	VehicleDesc() {}

	physx::PxFilterData chassisSimFilterData;  //word0 = collide type, word1 = collide against types, word2 = PxPairFlags
	physx::PxFilterData wheelSimFilterData;	//word0 = collide type, word1 = collide against types, word2 = PxPairFlags
};

static std::shared_ptr<pragma::physics::PhysXMaterial> gMaterial;
physx::PxVehicleDrive4W* createVehicle4W(
	pragma::physics::PhysXEnvironment &env,const VehicleDesc& vehicleDesc,
	const pragma::physics::VehicleCreateInfo &vhcCreateInfo,
	util::TSharedHandle<pragma::physics::PhysXRigidBody> &outRigidBody
)
{
	//const physx::PxF32 wheelWidth = wheelCreateInfo.width /gLengthScale;
	//const physx::PxF32 wheelRadius = wheelCreateInfo.radius /gLengthScale;
	const physx::PxU32 numWheels = vhcCreateInfo.wheels.size();

	const physx::PxFilterData& chassisSimFilterData = vehicleDesc.chassisSimFilterData;
	const physx::PxFilterData& wheelSimFilterData = vehicleDesc.wheelSimFilterData;

	//Construct a physx actor with shapes for the chassis and wheels.
	//Set the rigid body mass, moment of inertia, and center of mass offset.
	util::TSharedHandle<pragma::physics::PhysXRigidBody> veh4WActor = nullptr;
	{
		//Rigid body data.
		physx::PxVehicleChassisData rigidBodyData;
		rigidBodyData.mMOI = env.ToPhysXVector(vhcCreateInfo.chassis.momentOfInertia);
		rigidBodyData.mMass = vhcCreateInfo.chassis.mass;
		rigidBodyData.mCMOffset = env.ToPhysXVector(vhcCreateInfo.chassis.cmOffset) /gLengthScale;

		veh4WActor = createVehicleActor
		(rigidBodyData,
			vhcCreateInfo, wheelSimFilterData,
			 1, chassisSimFilterData,
			env.GetPhysics());
	}

	//Set up the sim data for the wheels.
	std::unique_ptr<physx::PxVehicleWheelsSimData,void(*)(physx::PxVehicleWheelsSimData*)> wheelsSimData {
		physx::PxVehicleWheelsSimData::allocate(numWheels),
		[](physx::PxVehicleWheelsSimData *simData) {
			simData->free();
		}
	};
	{
		//Compute the wheel center offsets from the origin.

		//Set up the simulation data for all wheels.
		setupWheelsSimulationData
		(env,vhcCreateInfo,
			env.ToPhysXVector(vhcCreateInfo.chassis.cmOffset) /gLengthScale, vhcCreateInfo.chassis.mass,
			wheelsSimData.get());
	}

	//Set up the sim data for the vehicle drive model.
	physx::PxVehicleDriveSimData4W driveSimData;
	{
		//Diff
		physx::PxVehicleDifferential4WData diff;
		switch(vhcCreateInfo.wheelDrive)
		{
		case pragma::physics::VehicleCreateInfo::WheelDrive::Four:
			diff.mType = physx::PxVehicleDifferential4WData::eDIFF_TYPE_LS_4WD;
			break;
		case pragma::physics::VehicleCreateInfo::WheelDrive::Front:
			diff.mType = physx::PxVehicleDifferential4WData::eDIFF_TYPE_LS_FRONTWD;
			break;
		case pragma::physics::VehicleCreateInfo::WheelDrive::Rear:
			diff.mType = physx::PxVehicleDifferential4WData::eDIFF_TYPE_LS_REARWD;
			break;
		}
		driveSimData.setDiffData(diff);

		//Engine
		physx::PxVehicleEngineData engine;
		engine.mPeakTorque=vhcCreateInfo.maxEngineTorque;
		engine.mMaxOmega=vhcCreateInfo.maxEngineRotationSpeed;
		driveSimData.setEngineData(engine);

		//Gears
		physx::PxVehicleGearsData gears;
		gears.mSwitchTime=vhcCreateInfo.gearSwitchTime;
		driveSimData.setGearsData(gears);

		//Clutch
		physx::PxVehicleClutchData clutch;
		clutch.mStrength=vhcCreateInfo.clutchStrength;
		driveSimData.setClutchData(clutch);

		//Ackermann steer accuracy
		physx::PxVehicleAckermannGeometryData ackermann;
		ackermann.mAccuracy=1.0f;
		ackermann.mAxleSeparation=
			wheelsSimData->getWheelCentreOffset(physx::PxVehicleDrive4WWheelOrder::eFRONT_LEFT).z-
			wheelsSimData->getWheelCentreOffset(physx::PxVehicleDrive4WWheelOrder::eREAR_LEFT).z;
		ackermann.mFrontWidth=
			wheelsSimData->getWheelCentreOffset(physx::PxVehicleDrive4WWheelOrder::eFRONT_RIGHT).x-
			wheelsSimData->getWheelCentreOffset(physx::PxVehicleDrive4WWheelOrder::eFRONT_LEFT).x;
		ackermann.mRearWidth=
			wheelsSimData->getWheelCentreOffset(physx::PxVehicleDrive4WWheelOrder::eREAR_RIGHT).x -
			wheelsSimData->getWheelCentreOffset(physx::PxVehicleDrive4WWheelOrder::eREAR_LEFT).x;
		driveSimData.setAckermannGeometryData(ackermann);
	}

	//Create a vehicle from the wheels and drive sim data.
	physx::PxVehicleDrive4W* vehDrive4W = physx::PxVehicleDrive4W::allocate(numWheels);
	vehDrive4W->setup(&env.GetPhysics(), &static_cast<physx::PxRigidDynamic&>(veh4WActor->GetInternalObject()), *wheelsSimData, driveSimData, numWheels - 4);
	//Configure the userdata
	outRigidBody = veh4WActor;
	return vehDrive4W;
}

snippetvehicle::VehicleSceneQueryData	gVehicleSceneQueryData;

physx::PxF32 gSteerVsForwardSpeedData[2*8]=
{
	0.0f,		0.75f,
	5.0f,		0.75f,
	30.0f,		0.125f,
	120.0f,		0.1f,
	PX_MAX_F32, PX_MAX_F32,
	PX_MAX_F32, PX_MAX_F32,
	PX_MAX_F32, PX_MAX_F32,
	PX_MAX_F32, PX_MAX_F32
};
physx::PxFixedSizeLookupTable<8> gSteerVsForwardSpeedTable(gSteerVsForwardSpeedData,4);
util::TSharedHandle<pragma::physics::IVehicle> pragma::physics::PhysXEnvironment::CreateVehicle(const VehicleCreateInfo &vhcDesc)
{
	gMaterial = std::dynamic_pointer_cast<PhysXMaterial>(CreateMaterial(0.5f,0.5f,0.6f));
	// Input
	pragma::physics::ChassisCreateInfo chassisCreateInfo = vhcDesc.chassis;
	std::vector<pragma::physics::WheelCreateInfo> wheelCreateInfos = vhcDesc.wheels;

	VehicleDesc vehicleDesc;
	vehicleDesc.chassisSimFilterData = physx::PxFilterData(COLLISION_FLAG_CHASSIS, COLLISION_FLAG_CHASSIS_AGAINST, 0, 0);
	vehicleDesc.wheelSimFilterData = physx::PxFilterData(COLLISION_FLAG_WHEEL, COLLISION_FLAG_WHEEL_AGAINST, 0, 0);


	// TODO: Move to init
	//Set up the friction values arising from combinations of tire type and surface type.
	std::vector<physx::PxVehicleDrivableSurfaceType> drivableSurfaceTypes {};
	std::vector<physx::PxMaterial*> materials;
	materials.push_back(&PhysXMaterial::GetMaterial(GetGenericMaterial()).GetInternalObject());
	drivableSurfaceTypes.push_back({});
	drivableSurfaceTypes.back().mType = 0;
	auto numTireTypes = 1;
	auto numSurfaceTypes = drivableSurfaceTypes.size();
	//m_surfaceTirePairs = px_create_unique_ptr(physx::PxVehicleDrivableSurfaceToTireFrictionPairs::allocate(numTireTypes,numSurfaceTypes));
	assert(materials.size() == numSurfaceTypes);
	//m_surfaceTirePairs->setup(numTireTypes,numSurfaceTypes,const_cast<const physx::PxMaterial**>(static_cast<physx::PxMaterial**>(materials.data())),drivableSurfaceTypes.data());//,);

	auto *gPhysics = &GetPhysics();
	auto *gCooking = &GetCooking();
	auto *gScene = &GetScene();
	auto *physics = gPhysics;
	auto *cooking = gCooking;


	///

	//Create the batched scene queries for the suspension raycasts.
	gVehicleSceneQueryData = snippetvehicle::VehicleSceneQueryData::allocate(1, PX_MAX_NB_WHEELS, 1, 1, snippetvehicle::WheelSceneQueryPreFilterBlocking, NULL);
	auto batchQuery = pragma::physics::px_create_unique_ptr(snippetvehicle::VehicleSceneQueryData::setUpBatchedSceneQuery(0, gVehicleSceneQueryData, gScene));

	//Create a vehicle that will drive on the plane.
	util::TSharedHandle<pragma::physics::PhysXRigidBody> rigidBody;
	auto gVehicle = px_create_unique_ptr<physx::PxVehicleDrive>(createVehicle4W(*this,vehicleDesc,vhcDesc,rigidBody));
	auto *gVehicle4W = static_cast<physx::PxVehicleDrive4W*>(gVehicle.get());


	//Convert the vehicle from meters to the chosen length scale.
	customizeVehicleToLengthScale(gLengthScale, gVehicle4W->getRigidDynamicActor(), &gVehicle4W->mWheelsSimData, &gVehicle4W->mDriveSimData);

	//Convert the steer angle vs forward speed table to the chosen length scale.
	for(physx::PxU32 i = 0; i < gSteerVsForwardSpeedTable.mNbDataPairs; i++)
	{
		gSteerVsForwardSpeedTable.mDataPairs[2*i +0] *= gLengthScale;
	}

	InitializeCollisionObject(*rigidBody);

	auto vhc = CreateSharedHandle<PhysXVehicle>(
		*this,std::move(gVehicle),util::shared_handle_cast<PhysXRigidBody,ICollisionObject>(rigidBody),
		std::move(batchQuery)
	);
	AddVehicle(*vhc);
	vhc->Spawn();
	vhc->SetRestState();
	return util::shared_handle_cast<PhysXVehicle,IVehicle>(vhc);
}
#pragma optimize("",on)
