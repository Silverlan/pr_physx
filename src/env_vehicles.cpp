#include "pr_physx/environment.hpp"
#include "pr_physx/vehicle.hpp"
#include "pr_physx/material.hpp"
#include "pr_physx/collision_object.hpp"
#include "pr_physx/shape.hpp"
#include <pragma/networkstate/networkstate.h>
#include <vehicle/PxVehicleSDK.h>

enum
{
	SURFACE_TYPE_MUD=0,
	SURFACE_TYPE_TARMAC,
	SURFACE_TYPE_SNOW,
	SURFACE_TYPE_GRASS,
	MAX_NUM_SURFACE_TYPES
};

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

	VehicleSceneQueryData::VehicleSceneQueryData()
		:  mNumQueriesPerBatch(0),
		mNumHitResultsPerQuery(0),
		mRaycastResults(NULL),
		mRaycastHitBuffer(NULL),
		mPreFilterShader(NULL),
		mPostFilterShader(NULL)
	{
	}

	VehicleSceneQueryData::~VehicleSceneQueryData()
	{
	}

	VehicleSceneQueryData* VehicleSceneQueryData::allocate
	(const PxU32 maxNumVehicles, const PxU32 maxNumWheelsPerVehicle, const PxU32 maxNumHitPointsPerWheel, const PxU32 numVehiclesInBatch, 
		PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader)
	{
		const PxU32 sqDataSize = ((sizeof(VehicleSceneQueryData) + 15) & ~15);

		const PxU32 maxNumWheels = maxNumVehicles*maxNumWheelsPerVehicle;
		const PxU32 raycastResultSize = ((sizeof(PxRaycastQueryResult)*maxNumWheels + 15) & ~15);
		const PxU32 sweepResultSize = ((sizeof(PxSweepQueryResult)*maxNumWheels + 15) & ~15);

		const PxU32 maxNumHitPoints = maxNumWheels*maxNumHitPointsPerWheel;
		const PxU32 raycastHitSize = ((sizeof(PxRaycastHit)*maxNumHitPoints + 15) & ~15);
		const PxU32 sweepHitSize = ((sizeof(PxSweepHit)*maxNumHitPoints + 15) & ~15);

		const PxU32 size = sqDataSize + raycastResultSize + raycastHitSize + sweepResultSize + sweepHitSize;
		auto *buffer = new PxU8[size];

		VehicleSceneQueryData* sqData = new(buffer) VehicleSceneQueryData();
		sqData->mNumQueriesPerBatch = numVehiclesInBatch*maxNumWheelsPerVehicle;
		sqData->mNumHitResultsPerQuery = maxNumHitPointsPerWheel;
		buffer += sqDataSize;

		sqData->mRaycastResults = reinterpret_cast<PxRaycastQueryResult*>(buffer);
		buffer += raycastResultSize;

		sqData->mRaycastHitBuffer = reinterpret_cast<PxRaycastHit*>(buffer);
		buffer += raycastHitSize;

		sqData->mSweepResults = reinterpret_cast<PxSweepQueryResult*>(buffer);
		buffer += sweepResultSize;

		sqData->mSweepHitBuffer = reinterpret_cast<PxSweepHit*>(buffer);
		buffer += sweepHitSize;

		for (PxU32 i = 0; i < maxNumWheels; i++)
		{
			new(sqData->mRaycastResults + i) PxRaycastQueryResult();
			new(sqData->mSweepResults + i) PxSweepQueryResult();
		}

		for (PxU32 i = 0; i < maxNumHitPoints; i++)
		{
			new(sqData->mRaycastHitBuffer + i) PxRaycastHit();
			new(sqData->mSweepHitBuffer + i) PxSweepHit();
		}

		sqData->mPreFilterShader = preFilterShader;
		sqData->mPostFilterShader = postFilterShader;

		return sqData;
	}

	void VehicleSceneQueryData::free(PxAllocatorCallback& allocator)
	{
		allocator.deallocate(this);
	}

	PxBatchQuery* VehicleSceneQueryData::setUpBatchedSceneQuery(const PxU32 batchId, const VehicleSceneQueryData& vehicleSceneQueryData, PxScene* scene)
	{
		const PxU32 maxNumQueriesInBatch =  vehicleSceneQueryData.mNumQueriesPerBatch;
		const PxU32 maxNumHitResultsInBatch = vehicleSceneQueryData.mNumQueriesPerBatch*vehicleSceneQueryData.mNumHitResultsPerQuery;

		PxBatchQueryDesc sqDesc(maxNumQueriesInBatch, maxNumQueriesInBatch, 0);

		sqDesc.queryMemory.userRaycastResultBuffer = vehicleSceneQueryData.mRaycastResults + batchId*maxNumQueriesInBatch;
		sqDesc.queryMemory.userRaycastTouchBuffer = vehicleSceneQueryData.mRaycastHitBuffer + batchId*maxNumHitResultsInBatch;
		sqDesc.queryMemory.raycastTouchBufferSize = maxNumHitResultsInBatch;

		sqDesc.queryMemory.userSweepResultBuffer = vehicleSceneQueryData.mSweepResults + batchId*maxNumQueriesInBatch;
		sqDesc.queryMemory.userSweepTouchBuffer = vehicleSceneQueryData.mSweepHitBuffer + batchId*maxNumHitResultsInBatch;
		sqDesc.queryMemory.sweepTouchBufferSize = maxNumHitResultsInBatch;

		sqDesc.preFilterShader = vehicleSceneQueryData.mPreFilterShader;

		sqDesc.postFilterShader = vehicleSceneQueryData.mPostFilterShader;

		return scene->createBatchQuery(sqDesc);
	}

	PxRaycastQueryResult* VehicleSceneQueryData::getRaycastQueryResultBuffer(const PxU32 batchId) 
	{
		return (mRaycastResults + batchId*mNumQueriesPerBatch);
	}

	PxSweepQueryResult* VehicleSceneQueryData::getSweepQueryResultBuffer(const PxU32 batchId)
	{
		return (mSweepResults + batchId*mNumQueriesPerBatch);
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

namespace snippetvehicle
{

	using namespace physx;

	//Tire model friction for each combination of drivable surface type and tire type.
	static PxF32 gTireFrictionMultipliers[MAX_NUM_SURFACE_TYPES][MAX_NUM_TIRE_TYPES]=
	{
		//NORMAL,	WORN
		{1.00f,		0.1f}//TARMAC
	};

	PxVehicleDrivableSurfaceToTireFrictionPairs* createFrictionPairs(const PxMaterial* defaultMaterial)
	{
		PxVehicleDrivableSurfaceType surfaceTypes[1];
		surfaceTypes[0].mType = SURFACE_TYPE_TARMAC;

		const PxMaterial* surfaceMaterials[1];
		surfaceMaterials[0] = defaultMaterial;

		PxVehicleDrivableSurfaceToTireFrictionPairs* surfaceTirePairs =
			PxVehicleDrivableSurfaceToTireFrictionPairs::allocate(MAX_NUM_TIRE_TYPES,MAX_NUM_SURFACE_TYPES);

		surfaceTirePairs->setup(MAX_NUM_TIRE_TYPES, MAX_NUM_SURFACE_TYPES, surfaceMaterials, surfaceTypes);

		for(PxU32 i = 0; i < MAX_NUM_SURFACE_TYPES; i++)
		{
			for(PxU32 j = 0; j < MAX_NUM_TIRE_TYPES; j++)
			{
				surfaceTirePairs->setTypePairFriction(i,j,gTireFrictionMultipliers[i][j]);
			}
		}
		return surfaceTirePairs;
	}

} // namespace snippetvehicle

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
	const std::vector<pragma::physics::WheelCreateInfo> &wheelDescs, const physx::PxFilterData& wheelSimFilterData,
	std::shared_ptr<pragma::physics::PhysXShape> &chassisShape, const physx::PxU32 numChassisMeshes, const physx::PxFilterData& chassisSimFilterData,
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

	//Add all the wheel shapes to the actor.
	std::vector<pragma::physics::IShape*> vhcShapes {};
	vhcShapes.reserve(wheelDescs.size());
	for(auto &wheelDesc : wheelDescs)
		vhcShapes.push_back(wheelDesc.shape.get());
	vhcShapes.push_back(chassisShape.get());


	auto compoundShape = chassisShape->GetPxEnv().CreateCompoundShape(vhcShapes);
	auto rigidBody = util::shared_handle_cast<pragma::physics::IRigidBody,pragma::physics::PhysXRigidBody>(chassisShape->GetPxEnv().CreateRigidBody(chassisData.mMass,*compoundShape,Vector3{},true));

	auto &actorShapes = rigidBody->GetActorShapeCollection().GetActorShapes();
	auto &chassisActorShape = actorShapes.back();
	auto &pxActorShape = chassisActorShape->GetActorShape();
	pxActorShape.setQueryFilterData(chassisQryFilterData);
	pxActorShape.setSimulationFilterData(chassisSimFilterData);
	pxActorShape.setLocalPose(physx::PxTransform(physx::PxIdentity));

	for(auto i=0;i<actorShapes.size() -1;++i)
	{
		auto &pxActorShape = actorShapes.at(i)->GetActorShape();
		pxActorShape.setQueryFilterData(wheelQryFilterData);
		pxActorShape.setSimulationFilterData(wheelSimFilterData);
		pxActorShape.setLocalPose(physx::PxTransform(physx::PxIdentity));
	}

	auto &dynamicRigidActor = static_cast<physx::PxRigidDynamic&>(rigidBody->GetInternalObject());
	dynamicRigidActor.setMass(chassisData.mMass);
	dynamicRigidActor.setMassSpaceInertiaTensor(chassisData.mMOI);
	dynamicRigidActor.setCMassLocalPose(physx::PxTransform(chassisData.mCMOffset,physx::PxQuat(physx::PxIdentity)));
	return rigidBody;
}

void setupWheelsSimulationData
(pragma::physics::PhysXEnvironment &env,const std::vector<pragma::physics::WheelCreateInfo> &wheelCreateInfos,
	const physx::PxVec3& chassisCMOffset, const physx::PxF32 chassisMass,
	physx::PxVehicleWheelsSimData* wheelsSimData)
{
	//Set up the wheels.
	struct WheelData
	{
		physx::PxVehicleWheelData data;
		physx::PxVehicleTireData tire;
		physx::PxVec3 centerActorOffset;
		//physx::PxVehicleSuspensionData suspension;

		//physx::PxVec3 suspTravelDirection;
		//physx::PxVec3 wheelCentreCMOffset;
		//physx::PxVec3 suspForceAppCMOffset;
		//physx::PxVec3 tireForceAppCMOffset;
	};
	std::vector<WheelData> wheels {};
	wheels.reserve(wheelCreateInfos.size());
	std::vector<physx::PxVec3> wheelCenterActorOffsets;
	for(auto &createInfo : wheelCreateInfos)
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

		wheel.centerActorOffset = env.ToPhysXVector(createInfo.chassisOffset) /gLengthScale;
		wheelCenterActorOffsets.push_back(wheel.centerActorOffset);
		//wheel.suspension = 
	}

	std::vector<physx::PxVehicleSuspensionData> suspensions {};
	suspensions.resize(wheels.size());
	{
		//Compute the mass supported by each suspension spring.
		physx::PxF32 suspSprungMasses[PX_MAX_NB_WHEELS];
		physx::PxVehicleComputeSprungMasses
		(wheelCreateInfos.size(), wheelCenterActorOffsets.data(), 
			chassisCMOffset, chassisMass, 1, suspSprungMasses);

		//Set the suspension data.
		for(physx::PxU32 i = 0; i < wheelCreateInfos.size(); i++)
		{
			suspensions[i].mMaxCompression = 0.3f;
			suspensions[i].mMaxDroop = 0.1f;
			suspensions[i].mSpringStrength = 35000.0f;	
			suspensions[i].mSpringDamperRate = 4500.0f;
			suspensions[i].mSprungMass = suspSprungMasses[i];
		}

		//Set the camber angles.
		const physx::PxF32 camberAngleAtRest=0.0;
		const physx::PxF32 camberAngleAtMaxDroop=0.01f;
		const physx::PxF32 camberAngleAtMaxCompression=-0.01f;
		for(physx::PxU32 i = 0; i < wheelCreateInfos.size(); i+=2)
		{
			suspensions[i + 0].mCamberAtRest =  camberAngleAtRest;
			suspensions[i + 1].mCamberAtRest =  -camberAngleAtRest;
			suspensions[i + 0].mCamberAtMaxDroop = camberAngleAtMaxDroop;
			suspensions[i + 1].mCamberAtMaxDroop = -camberAngleAtMaxDroop;
			suspensions[i + 0].mCamberAtMaxCompression = camberAngleAtMaxCompression;
			suspensions[i + 1].mCamberAtMaxCompression = -camberAngleAtMaxCompression;
		}
	}

	//Set up the wheel geometry.
	physx::PxVec3 suspTravelDirections[PX_MAX_NB_WHEELS];
	physx::PxVec3 wheelCentreCMOffsets[PX_MAX_NB_WHEELS];
	physx::PxVec3 suspForceAppCMOffsets[PX_MAX_NB_WHEELS];
	physx::PxVec3 tireForceAppCMOffsets[PX_MAX_NB_WHEELS];
	{
		//Set the geometry data.
		for(physx::PxU32 i = 0; i < wheelCreateInfos.size(); i++)
		{
			//Vertical suspension travel.
			suspTravelDirections[i] = physx::PxVec3(0,-1,0);

			//Wheel center offset is offset from rigid body center of mass.
			wheelCentreCMOffsets[i] = 
				wheelCenterActorOffsets[i] - chassisCMOffset;

			//Suspension force application point 0.3 metres below 
			//rigid body center of mass.
			suspForceAppCMOffsets[i] =
				physx::PxVec3(wheelCentreCMOffsets[i].x,-0.3f,wheelCentreCMOffsets[i].z);

			//Tire force application point 0.3 metres below 
			//rigid body center of mass.
			tireForceAppCMOffsets[i] =
				physx::PxVec3(wheelCentreCMOffsets[i].x,-0.3f,wheelCentreCMOffsets[i].z);
		}
	}

	//Set up the filter data of the raycast that will be issued by each suspension.
	physx::PxFilterData qryFilterData;
	setupNonDrivableSurface(qryFilterData);

	//Set the wheel, tire and suspension data.
	//Set the geometry data.
	//Set the query filter data
	for(physx::PxU32 i = 0; i < wheelCreateInfos.size(); i++)
	{
		auto &wheel = wheels.at(i);
		wheelsSimData->setWheelData(i, wheel.data);
		wheelsSimData->setTireData(i, wheel.tire);
		wheelsSimData->setSuspensionData(i, suspensions[i]);
		wheelsSimData->setSuspTravelDirection(i, suspTravelDirections[i]);
		wheelsSimData->setWheelCentreOffset(i, wheelCentreCMOffsets[i]);
		wheelsSimData->setSuspForceAppPointOffset(i, suspForceAppCMOffsets[i]);
		wheelsSimData->setTireForceAppPointOffset(i, tireForceAppCMOffsets[i]);
		wheelsSimData->setSceneQueryFilterData(i, qryFilterData);
		wheelsSimData->setWheelShapeMapping(i, physx::PxI32(i)); 
	}

	//Add a front and rear anti-roll bar
	physx::PxVehicleAntiRollBarData barFront;
	barFront.mWheel0 = physx::PxVehicleDrive4WWheelOrder::eFRONT_LEFT;
	barFront.mWheel1 = physx::PxVehicleDrive4WWheelOrder::eFRONT_RIGHT;
	barFront.mStiffness = 10000.0f;
	wheelsSimData->addAntiRollBarData(barFront);
	physx::PxVehicleAntiRollBarData barRear;
	barRear.mWheel0 = physx::PxVehicleDrive4WWheelOrder::eREAR_LEFT;
	barRear.mWheel1 = physx::PxVehicleDrive4WWheelOrder::eREAR_RIGHT;
	barRear.mStiffness = 10000.0f;
	wheelsSimData->addAntiRollBarData(barRear);
}

struct ActorUserData
{
	ActorUserData()
		: vehicle(NULL),
		actor(NULL)
	{
	}

	const physx::PxVehicleWheels* vehicle;
	const physx::PxActor* actor;
};

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
	VehicleDesc()
		: 
		actorUserData(NULL),
		shapeUserDatas(NULL)
	{
	}

	physx::PxFilterData chassisSimFilterData;  //word0 = collide type, word1 = collide against types, word2 = PxPairFlags
	physx::PxFilterData wheelSimFilterData;	//word0 = collide type, word1 = collide against types, word2 = PxPairFlags

	ActorUserData* actorUserData;
	ShapeUserData* shapeUserDatas;
};

void configureUserData(physx::PxVehicleWheels* vehicle, ActorUserData* actorUserData, ShapeUserData* shapeUserDatas)
{
	if(actorUserData)
	{
		vehicle->getRigidDynamicActor()->userData = actorUserData;
		actorUserData->vehicle = vehicle;
	}

	if(shapeUserDatas)
	{
		physx::PxShape* shapes[PX_MAX_NB_WHEELS + 1];
		vehicle->getRigidDynamicActor()->getShapes(shapes, PX_MAX_NB_WHEELS + 1);
		for(physx::PxU32 i = 0; i < vehicle->mWheelsSimData.getNbWheels(); i++)
		{
			const physx::PxI32 shapeId = vehicle->mWheelsSimData.getWheelShapeMapping(i);
			shapes[shapeId]->userData = &shapeUserDatas[i];
			shapeUserDatas[i].isWheel = true;
			shapeUserDatas[i].wheelId = i;
		}
	}
}

static std::shared_ptr<pragma::physics::PhysXMaterial> gMaterial;
physx::PxVehicleDrive4W* createVehicle4W(
	pragma::physics::PhysXEnvironment &env,const VehicleDesc& vehicleDesc, const pragma::physics::ChassisCreateInfo &chassisCreateInfo, 
	
	const std::vector<pragma::physics::WheelCreateInfo> &wheelCreateInfos, 
	
	physx::PxPhysics* physics,
	util::TSharedHandle<pragma::physics::PhysXRigidBody> &outRigidBody
)
{
	//const physx::PxF32 wheelWidth = wheelCreateInfo.width /gLengthScale;
	//const physx::PxF32 wheelRadius = wheelCreateInfo.radius /gLengthScale;
	const physx::PxU32 numWheels = wheelCreateInfos.size();

	const physx::PxFilterData& chassisSimFilterData = vehicleDesc.chassisSimFilterData;
	const physx::PxFilterData& wheelSimFilterData = vehicleDesc.wheelSimFilterData;

	//Construct a physx actor with shapes for the chassis and wheels.
	//Set the rigid body mass, moment of inertia, and center of mass offset.
	util::TSharedHandle<pragma::physics::PhysXRigidBody> veh4WActor = nullptr;
	{
		//Chassis just has a single convex shape for simplicity.
		physx::PxMaterial* chassisMaterials[1] = {&pragma::physics::PhysXMaterial::GetMaterial(*gMaterial).GetInternalObject()};
		auto chassisConvexMesh = chassisCreateInfo.shape;//createChassisMesh(env,chassisDims*gLengthScale,*pragma::physics::PhysXEnvironment::GetMaterial(*chassisMaterials[0]));

		//Rigid body data.
		physx::PxVehicleChassisData rigidBodyData;
		rigidBodyData.mMOI = env.ToPhysXVector(chassisCreateInfo.momentOfInertia);
		rigidBodyData.mMass = chassisCreateInfo.mass;
		rigidBodyData.mCMOffset = env.ToPhysXVector(chassisCreateInfo.cmOffset) /gLengthScale;

		veh4WActor = createVehicleActor
		(rigidBodyData,
			wheelCreateInfos, wheelSimFilterData,
			std::dynamic_pointer_cast<pragma::physics::PhysXShape>(chassisConvexMesh), 1, chassisSimFilterData,
			*physics);
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
		(env,wheelCreateInfos,
			env.ToPhysXVector(chassisCreateInfo.cmOffset) /gLengthScale, chassisCreateInfo.mass,
			wheelsSimData.get());
	}

	//Set up the sim data for the vehicle drive model.
	physx::PxVehicleDriveSimData4W driveSimData;
	{
		//Diff
		physx::PxVehicleDifferential4WData diff;
		diff.mType=physx::PxVehicleDifferential4WData::eDIFF_TYPE_LS_4WD;
		driveSimData.setDiffData(diff);

		//Engine
		physx::PxVehicleEngineData engine;
		engine.mPeakTorque=500.0f;
		engine.mMaxOmega=600.0f;//approx 6000 rpm
		driveSimData.setEngineData(engine);

		//Gears
		physx::PxVehicleGearsData gears;
		gears.mSwitchTime=0.5f;
		driveSimData.setGearsData(gears);

		//Clutch
		physx::PxVehicleClutchData clutch;
		clutch.mStrength=10.0f;
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
	vehDrive4W->setup(physics, &static_cast<physx::PxRigidDynamic&>(veh4WActor->GetInternalObject()), *wheelsSimData, driveSimData, numWheels - 4);
	//Configure the userdata
	configureUserData(vehDrive4W, vehicleDesc.actorUserData, vehicleDesc.shapeUserDatas);
	outRigidBody = veh4WActor;
	return vehDrive4W;
}

snippetvehicle::VehicleSceneQueryData*	gVehicleSceneQueryData = NULL;

physx::PxVehicleDrivableSurfaceToTireFrictionPairs* gFrictionPairs = NULL;

static physx::PxF32 gTireFrictionMultipliers[MAX_NUM_SURFACE_TYPES][MAX_NUM_TIRE_TYPES]=
{
	//NORMAL,	WORN
	{1.00f,		0.1f},//TARMAC
	{1.00f,		0.1f}//TARMAC
};

physx::PxVehicleDrivableSurfaceToTireFrictionPairs* createFrictionPairs(const physx::PxMaterial* defaultMaterial,const physx::PxMaterial* defaultMaterial2)
{
	physx::PxVehicleDrivableSurfaceType surfaceTypes[2];
	surfaceTypes[0].mType = SURFACE_TYPE_TARMAC;
	surfaceTypes[1].mType = SURFACE_TYPE_TARMAC;

	const physx::PxMaterial* surfaceMaterials[2];
	surfaceMaterials[0] = defaultMaterial;
	surfaceMaterials[1] = defaultMaterial2;

	physx::PxVehicleDrivableSurfaceToTireFrictionPairs* surfaceTirePairs =
		physx::PxVehicleDrivableSurfaceToTireFrictionPairs::allocate(MAX_NUM_TIRE_TYPES,MAX_NUM_SURFACE_TYPES);

	surfaceTirePairs->setup(MAX_NUM_TIRE_TYPES, MAX_NUM_SURFACE_TYPES, surfaceMaterials, surfaceTypes);

	for(physx::PxU32 i = 0; i < MAX_NUM_SURFACE_TYPES; i++)
	{
		for(physx::PxU32 j = 0; j < MAX_NUM_TIRE_TYPES; j++)
		{
			surfaceTirePairs->setTypePairFriction(i,j,gTireFrictionMultipliers[i][j]);
		}
	}
	return surfaceTirePairs;
}

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
util::TSharedHandle<pragma::physics::IVehicle> pragma::physics::PhysXEnvironment::CreateVehicle(const ChassisCreateInfo &chassisDesc,const std::vector<WheelCreateInfo> &wheelDescs)
{
	gMaterial = std::dynamic_pointer_cast<PhysXMaterial>(CreateMaterial(0.5f,0.5f,0.6f));
	// Input
	pragma::physics::ChassisCreateInfo chassisCreateInfo = chassisDesc;
	std::vector<pragma::physics::WheelCreateInfo> wheelCreateInfos = wheelDescs;

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
	auto batchQuery = pragma::physics::px_create_unique_ptr(snippetvehicle::VehicleSceneQueryData::setUpBatchedSceneQuery(0, *gVehicleSceneQueryData, gScene));

	//Create the friction table for each combination of tire and surface type.
	gFrictionPairs = createFrictionPairs(&gMaterial->GetInternalObject(),&PhysXMaterial::GetMaterial(GetGenericMaterial()).GetInternalObject());

	//Create a vehicle that will drive on the plane.
	util::TSharedHandle<pragma::physics::PhysXRigidBody> rigidBody;
	auto gVehicle = px_create_unique_ptr<physx::PxVehicleDrive>(createVehicle4W(*this,vehicleDesc,chassisCreateInfo,wheelCreateInfos, gPhysics,rigidBody));
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
	vhc->SetGear(pragma::physics::IVehicle::Gear::First);
	vhc->SetUseAutoGears(true);
	vhc->SetBrakeFactor(1.f);
	vhc->SetUseDigitalInputs(true);
	return util::shared_handle_cast<PhysXVehicle,IVehicle>(vhc);
}
#pragma optimize("",on)
