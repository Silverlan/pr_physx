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

	PxQueryHitType::Enum WheelSceneQueryPostFilterBlocking
	(PxFilterData filterData0, PxFilterData filterData1,
		const void* constantBlock, PxU32 constantBlockSize,
		const PxQueryHit& hit)
	{
		PX_UNUSED(filterData0);
		PX_UNUSED(filterData1);
		PX_UNUSED(constantBlock);
		PX_UNUSED(constantBlockSize);
		if((static_cast<const PxSweepHit&>(hit)).hadInitialOverlap())
			return PxQueryHitType::eNONE;
		return PxQueryHitType::eBLOCK;
	}

	PxQueryHitType::Enum WheelSceneQueryPreFilterNonBlocking
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
		return ((0 == (filterData1.word3 & DRIVABLE_SURFACE)) ? PxQueryHitType::eNONE : PxQueryHitType::eTOUCH);
	}

	PxQueryHitType::Enum WheelSceneQueryPostFilterNonBlocking
	(PxFilterData filterData0, PxFilterData filterData1,
		const void* constantBlock, PxU32 constantBlockSize,
		const PxQueryHit& hit)
	{
		PX_UNUSED(filterData0);
		PX_UNUSED(filterData1);
		PX_UNUSED(constantBlock);
		PX_UNUSED(constantBlockSize);
		if ((static_cast<const PxSweepHit&>(hit)).hadInitialOverlap())
			return PxQueryHitType::eNONE;
		return PxQueryHitType::eTOUCH;
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


enum
{
	DRIVABLE_SURFACE = 64,
	UNDRIVABLE_SURFACE = 32
};

enum
{
	COLLISION_FLAG_GROUND			=	1 << 0,
	COLLISION_FLAG_WHEEL			=	1 << 1,
	COLLISION_FLAG_CHASSIS			=	1 << 2,
	COLLISION_FLAG_OBSTACLE			=	1 << 3,
	COLLISION_FLAG_DRIVABLE_OBSTACLE=	1 << 4,

	COLLISION_FLAG_GROUND_AGAINST	=															COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE | COLLISION_FLAG_DRIVABLE_OBSTACLE,
	COLLISION_FLAG_WHEEL_AGAINST	=									COLLISION_FLAG_WHEEL |	COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE,
	COLLISION_FLAG_CHASSIS_AGAINST	=			COLLISION_FLAG_GROUND | COLLISION_FLAG_WHEEL |	COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE | COLLISION_FLAG_DRIVABLE_OBSTACLE,
	COLLISION_FLAG_OBSTACLE_AGAINST	=			COLLISION_FLAG_GROUND | COLLISION_FLAG_WHEEL |	COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE | COLLISION_FLAG_DRIVABLE_OBSTACLE,
	COLLISION_FLAG_DRIVABLE_OBSTACLE_AGAINST=	COLLISION_FLAG_GROUND 						 |	COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE | COLLISION_FLAG_DRIVABLE_OBSTACLE,
};

namespace pragma::physics
{
	struct WheelCreateInfo
	{
		float mass;
		float MOI;
		float radius;
		float width;
		Vector3 centerOffset;
		float dampingRate = 0.f;

		float maxBrakeTorque;
		float maxHandBrakeTorque;
		float maxSteerAngle;
	};

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

	struct ChassisCreateInfo
	{
		float mass;
		Vector3 CMOffset;
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

	//Scale the collision meshes too.
	{
		physx::PxShape* shapes[16];
		const physx::PxU32 nbShapes = rigidDynamic->getShapes(shapes, 16);
		for(physx::PxU32 i = 0; i < nbShapes; i++)
		{
			switch(shapes[i]->getGeometryType())
			{
			case physx::PxGeometryType::eSPHERE:
			{
				physx::PxSphereGeometry sphere;
				shapes[i]->getSphereGeometry(sphere);
				sphere.radius *= lengthScale;
				shapes[i]->setGeometry(sphere);
			}
			break;
			case physx::PxGeometryType::ePLANE:
				PX_ASSERT(false);
				break;
			case physx::PxGeometryType::eCAPSULE:
			{
				physx::PxCapsuleGeometry capsule;
				shapes[i]->getCapsuleGeometry(capsule);
				capsule.radius *= lengthScale;
				capsule.halfHeight*= lengthScale;
				shapes[i]->setGeometry(capsule);
			}
			break;
			case physx::PxGeometryType::eBOX:
			{
				physx::PxBoxGeometry box;
				shapes[i]->getBoxGeometry(box);
				box.halfExtents *= lengthScale;
				shapes[i]->setGeometry(box);
			}
			break;
			case physx::PxGeometryType::eCONVEXMESH:
			{
				physx::PxConvexMeshGeometry convexMesh;
				shapes[i]->getConvexMeshGeometry(convexMesh);
				convexMesh.scale.scale *= lengthScale;
				shapes[i]->setGeometry(convexMesh);
			}
			break;
			case physx::PxGeometryType::eTRIANGLEMESH:
			{
				physx::PxTriangleMeshGeometry triMesh;
				shapes[i]->getTriangleMeshGeometry(triMesh);
				triMesh.scale.scale *= lengthScale;
				shapes[i]->setGeometry(triMesh);
			}
			break;
			case physx::PxGeometryType::eHEIGHTFIELD:
			{
				physx::PxHeightFieldGeometry hf;
				shapes[i]->getHeightFieldGeometry(hf);
				hf.columnScale *= lengthScale;
				hf.heightScale *= lengthScale;
				hf.rowScale *= lengthScale;
				shapes[i]->setGeometry(hf);
			}
			break;
			case physx::PxGeometryType::eINVALID:
			case physx::PxGeometryType::eGEOMETRY_COUNT:
				break;
			}
		}
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

physx::PxVehicleWheelData pragma::physics::PhysXEnvironment::ToPxWheelData(const WheelCreateInfo &createInfo)
{
	physx::PxVehicleWheelData pxWheelData {};
	pxWheelData.mRadius = ToPhysXLength(createInfo.radius);
	pxWheelData.mWidth = ToPhysXLength(createInfo.width); // TODO
	pxWheelData.mMass = createInfo.mass;
	pxWheelData.mMOI = createInfo.MOI;
	pxWheelData.mDampingRate = createInfo.dampingRate; // TODO
	pxWheelData.mMaxBrakeTorque = createInfo.maxBrakeTorque; // TODO
	pxWheelData.mMaxHandBrakeTorque = createInfo.maxHandBrakeTorque; // TODO
	pxWheelData.mMaxSteer = createInfo.maxSteerAngle; // TODO
	pxWheelData.mToeAngle = 0; // TODO
	return pxWheelData;
}
physx::PxVehicleTireData pragma::physics::PhysXEnvironment::ToPxTireData(const TireCreateInfo &createInfo)
{
	physx::PxVehicleTireData pxTireData {};
	// TODO
	return pxTireData;
}

static void setupNonDrivableSurface(physx::PxFilterData& filterData)
{
	filterData.word3 = UNDRIVABLE_SURFACE;
}

std::shared_ptr<pragma::physics::IConvexHullShape> createChassisMesh(pragma::physics::PhysXEnvironment &env,const physx::PxVec3 dims,pragma::physics::IMaterial &mat)
{
	const physx::PxF32 x = dims.x*0.5f;
	const physx::PxF32 y = dims.y*0.5f;
	const physx::PxF32 z = dims.z*0.5f;
	auto shape = env.CreateConvexHullShape(mat);
	shape->ReservePoints(8);
	shape->AddPoint(Vector3(x,y,-z));
	shape->AddPoint(Vector3(x,y,z));
	shape->AddPoint(Vector3(x,-y,z));
	shape->AddPoint(Vector3(x,-y,-z));
	shape->AddPoint(Vector3(-x,y,-z)); 
	shape->AddPoint(Vector3(-x,y,z));
	shape->AddPoint(Vector3(-x,-y,z));
	shape->AddPoint(Vector3(-x,-y,-z));
	shape->Build();
	return shape;
}

std::shared_ptr<pragma::physics::IConvexHullShape> createWheelMesh(pragma::physics::PhysXEnvironment &env,const physx::PxF32 width, const physx::PxF32 radius,pragma::physics::IMaterial &mat)
{
	auto shape = env.CreateConvexHullShape(mat);
	shape->ReservePoints(32);
	for(physx::PxU32 i = 0; i < 16; i++)
	{
		const physx::PxF32 cosTheta = physx::PxCos(i*physx::PxPi*2.0f/16.0f);
		const physx::PxF32 sinTheta = physx::PxSin(i*physx::PxPi*2.0f/16.0f);
		const physx::PxF32 y = radius*cosTheta;
		const physx::PxF32 z = radius*sinTheta;
		shape->AddPoint(Vector3{-width/2.0f, y, z});
		shape->AddPoint(Vector3{+width/2.0f, y, z});
	}
	shape->Build();
	return shape;
}

util::TSharedHandle<pragma::physics::PhysXRigidBody> createVehicleActor
(const physx::PxVehicleChassisData& chassisData,
	std::shared_ptr<pragma::physics::IConvexHullShape>* wheelConvexMeshes, const physx::PxU32 numWheels, const physx::PxFilterData& wheelSimFilterData,
	std::shared_ptr<pragma::physics::PhysXShape> &chassisShape, const physx::PxU32 numChassisMeshes, const physx::PxFilterData& chassisSimFilterData,
	physx::PxPhysics& physics)
{
	//We need a rigid body actor for the vehicle.
	//Don't forget to add the actor to the scene after setting up the associated vehicle.

	//Add the chassis shapes to the actor.
	auto *mat = pragma::physics::PhysXShape::GetShape(*chassisShape).GetMaterial();
	if(mat == nullptr)
		mat = &chassisShape->GetPxEnv().GetGenericMaterial();
	std::vector<pragma::physics::IShape*> vhcShapes {};
	chassisShape->SetMaterial(*mat);
	/*auto actorShape = chassisShape->GetPxEnv().CreateActorShape(
		*physx::PxRigidActorExt::createExclusiveShape(
			*vehActor,pragma::physics::PhysXShape::GetShape(*chassisShape).GetInternalObject().any(),pragma::physics::PhysXMaterial::GetMaterial(*mat).GetInternalObject()
		)
	);
	physx::PxRigidDynamic* vehActor = physics.createRigidDynamic(physx::PxTransform(physx::PxIdentity));
	*/


	

	//Wheel and chassis query filter data.
	//Optional: cars don't drive on other cars.
	physx::PxFilterData wheelQryFilterData;
	setupNonDrivableSurface(wheelQryFilterData);
	physx::PxFilterData chassisQryFilterData;
	setupNonDrivableSurface(chassisQryFilterData);

	//Add all the wheel shapes to the actor.
	for(physx::PxU32 i = 0; i < numWheels; i++)
	{
		auto wheelShape = std::dynamic_pointer_cast<pragma::physics::PhysXConvexHullShape>(wheelConvexMeshes[i]);
		auto *mat = pragma::physics::PhysXShape::GetShape(*wheelShape).GetMaterial();
		if(mat == nullptr)
			mat = &wheelShape->GetPxEnv().GetGenericMaterial();
		wheelShape->SetMaterial(*mat);
		vhcShapes.push_back(wheelShape.get());
	}
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
	dynamicRigidActor.setMass(chassisData.mMass); // Scale?
	dynamicRigidActor.setMassSpaceInertiaTensor(chassisData.mMOI); // Scale?
	dynamicRigidActor.setCMassLocalPose(physx::PxTransform(chassisData.mCMOffset,physx::PxQuat(physx::PxIdentity)));
	return rigidBody;
}

void computeWheelCenterActorOffsets4W(const physx::PxF32 wheelFrontZ, const physx::PxF32 wheelRearZ, const physx::PxVec3& chassisDims, const physx::PxF32 wheelWidth, const physx::PxF32 wheelRadius, const physx::PxU32 numWheels, physx::PxVec3* wheelCentreOffsets)
{
	//chassisDims.z is the distance from the rear of the chassis to the front of the chassis.
	//The front has z = 0.5*chassisDims.z and the rear has z = -0.5*chassisDims.z.
	//Compute a position for the front wheel and the rear wheel along the z-axis.
	//Compute the separation between each wheel along the z-axis.
	const physx::PxF32 numLeftWheels = numWheels/2.0f;
	const physx::PxF32 deltaZ = (wheelFrontZ - wheelRearZ)/(numLeftWheels-1.0f);
	//Set the outside of the left and right wheels to be flush with the chassis.
	//Set the top of the wheel to be just touching the underside of the chassis.
	//Begin by setting the rear-left/rear-right/front-left,front-right wheels.
	wheelCentreOffsets[physx::PxVehicleDrive4WWheelOrder::eREAR_LEFT] = physx::PxVec3((-chassisDims.x + wheelWidth)*0.5f, -(chassisDims.y/2 + wheelRadius), wheelRearZ + 0*deltaZ*0.5f);
	wheelCentreOffsets[physx::PxVehicleDrive4WWheelOrder::eREAR_RIGHT] = physx::PxVec3((+chassisDims.x - wheelWidth)*0.5f, -(chassisDims.y/2 + wheelRadius), wheelRearZ + 0*deltaZ*0.5f);
	wheelCentreOffsets[physx::PxVehicleDrive4WWheelOrder::eFRONT_LEFT] = physx::PxVec3((-chassisDims.x + wheelWidth)*0.5f, -(chassisDims.y/2 + wheelRadius), wheelRearZ + (numLeftWheels-1)*deltaZ);
	wheelCentreOffsets[physx::PxVehicleDrive4WWheelOrder::eFRONT_RIGHT] = physx::PxVec3((+chassisDims.x - wheelWidth)*0.5f, -(chassisDims.y/2 + wheelRadius), wheelRearZ + (numLeftWheels-1)*deltaZ);
	//Set the remaining wheels.
	for(physx::PxU32 i = 2, wheelCount = 4; i < numWheels-2; i+=2, wheelCount+=2)
	{
		wheelCentreOffsets[wheelCount + 0] = physx::PxVec3((-chassisDims.x + wheelWidth)*0.5f, -(chassisDims.y/2 + wheelRadius), wheelRearZ + i*deltaZ*0.5f);
		wheelCentreOffsets[wheelCount + 1] = physx::PxVec3((+chassisDims.x - wheelWidth)*0.5f, -(chassisDims.y/2 + wheelRadius), wheelRearZ + i*deltaZ*0.5f);
	}
}

void setupWheelsSimulationData
(const physx::PxF32 wheelMass, const physx::PxF32 wheelMOI, const physx::PxF32 wheelRadius, const physx::PxF32 wheelWidth, 
	const physx::PxU32 numWheels, const physx::PxVec3* wheelCenterActorOffsets,
	const physx::PxVec3& chassisCMOffset, const physx::PxF32 chassisMass,
	physx::PxVehicleWheelsSimData* wheelsSimData)
{
	//Set up the wheels.
	physx::PxVehicleWheelData wheels[PX_MAX_NB_WHEELS];
	{
		//Set up the wheel data structures with mass, moi, radius, width.
		for(physx::PxU32 i = 0; i < numWheels; i++)
		{
			wheels[i].mMass = wheelMass;
			wheels[i].mMOI = wheelMOI;
			wheels[i].mRadius = wheelRadius;
			wheels[i].mWidth = wheelWidth;
		}

		//Enable the handbrake for the rear wheels only.
		wheels[physx::PxVehicleDrive4WWheelOrder::eREAR_LEFT].mMaxHandBrakeTorque=4000.0f;
		wheels[physx::PxVehicleDrive4WWheelOrder::eREAR_RIGHT].mMaxHandBrakeTorque=4000.0f;
		//Enable steering for the front wheels only.
		wheels[physx::PxVehicleDrive4WWheelOrder::eFRONT_LEFT].mMaxSteer=physx::PxPi*0.3333f;
		wheels[physx::PxVehicleDrive4WWheelOrder::eFRONT_RIGHT].mMaxSteer=physx::PxPi*0.3333f;
	}

	//Set up the tires.
	physx::PxVehicleTireData tires[PX_MAX_NB_WHEELS];
	{
		//Set up the tires.
		for(physx::PxU32 i = 0; i < numWheels; i++)
		{
			tires[i].mType = TIRE_TYPE_NORMAL;
		}
	}

	//Set up the suspensions
	physx::PxVehicleSuspensionData suspensions[PX_MAX_NB_WHEELS];
	{
		//Compute the mass supported by each suspension spring.
		physx::PxF32 suspSprungMasses[PX_MAX_NB_WHEELS];
		physx::PxVehicleComputeSprungMasses
		(numWheels, wheelCenterActorOffsets, 
			chassisCMOffset, chassisMass, 1, suspSprungMasses);

		//Set the suspension data.
		for(physx::PxU32 i = 0; i < numWheels; i++)
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
		for(physx::PxU32 i = 0; i < numWheels; i+=2)
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
		for(physx::PxU32 i = 0; i < numWheels; i++)
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
	for(physx::PxU32 i = 0; i < numWheels; i++)
	{
		wheelsSimData->setWheelData(i, wheels[i]);
		wheelsSimData->setTireData(i, tires[i]);
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
		: chassisMass(0.0f),
		chassisDims(physx::PxVec3(0.0f, 0.0f, 0.0f)),
		chassisMOI(physx::PxVec3(0.0f, 0.0f, 0.0f)),
		chassisCMOffset(physx::PxVec3(0.0f, 0.0f, 0.0f)),
		chassisMaterial(NULL),
		wheelMass(0.0f),
		wheelWidth(0.0f),
		wheelRadius(0.0f),
		wheelMOI(0.0f),
		wheelMaterial(NULL),
		actorUserData(NULL),
		shapeUserDatas(NULL)
	{
	}

	physx::PxF32 chassisMass;
	physx::PxVec3 chassisDims;
	physx::PxVec3 chassisMOI;
	physx::PxVec3 chassisCMOffset;
	physx::PxMaterial* chassisMaterial;
	physx::PxFilterData chassisSimFilterData;  //word0 = collide type, word1 = collide against types, word2 = PxPairFlags

	physx::PxF32 wheelMass;
	physx::PxF32 wheelWidth;
	physx::PxF32 wheelRadius;
	physx::PxF32 wheelMOI;
	physx::PxMaterial* wheelMaterial;
	physx::PxU32 numWheels;
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

physx::PxVehicleDrive4W* createVehicle4W(
	pragma::physics::PhysXEnvironment &env,const VehicleDesc& vehicleDesc, physx::PxPhysics* physics, physx::PxCooking* cooking,
	util::TSharedHandle<pragma::physics::PhysXRigidBody> &outRigidBody
)
{
	const physx::PxVec3 chassisDims = vehicleDesc.chassisDims;
	const physx::PxF32 wheelWidth = vehicleDesc.wheelWidth;
	const physx::PxF32 wheelRadius = vehicleDesc.wheelRadius;
	const physx::PxU32 numWheels = vehicleDesc.numWheels;

	const physx::PxFilterData& chassisSimFilterData = vehicleDesc.chassisSimFilterData;
	const physx::PxFilterData& wheelSimFilterData = vehicleDesc.wheelSimFilterData;

	//Construct a physx actor with shapes for the chassis and wheels.
	//Set the rigid body mass, moment of inertia, and center of mass offset.
	util::TSharedHandle<pragma::physics::PhysXRigidBody> veh4WActor = nullptr;
	{
		//Construct a convex mesh for a cylindrical wheel.
		auto wheelMesh = createWheelMesh(env, wheelWidth, wheelRadius, *pragma::physics::PhysXEnvironment::GetMaterial(*vehicleDesc.wheelMaterial));
		//Assume all wheels are identical for simplicity.
		std::array<std::shared_ptr<pragma::physics::IConvexHullShape>,PX_MAX_NB_WHEELS> wheelConvexMeshes;
		physx::PxMaterial* wheelMaterials[PX_MAX_NB_WHEELS];

		//Set the meshes and materials for the driven wheels.
		for(physx::PxU32 i = physx::PxVehicleDrive4WWheelOrder::eFRONT_LEFT; i <= physx::PxVehicleDrive4WWheelOrder::eREAR_RIGHT; i++)
		{
			wheelConvexMeshes[i] = wheelMesh;
			wheelMaterials[i] = vehicleDesc.wheelMaterial;
		}
		//Set the meshes and materials for the non-driven wheels
		for(physx::PxU32 i = physx::PxVehicleDrive4WWheelOrder::eREAR_RIGHT + 1; i < numWheels; i++)
		{
			wheelConvexMeshes[i] = wheelMesh;
			wheelMaterials[i] = vehicleDesc.wheelMaterial;
		}

		//Chassis just has a single convex shape for simplicity.
		physx::PxMaterial* chassisMaterials[1] = {vehicleDesc.chassisMaterial};
		auto chassisConvexMesh = createChassisMesh(env,chassisDims,*pragma::physics::PhysXEnvironment::GetMaterial(*chassisMaterials[0]));

		//Rigid body data.
		physx::PxVehicleChassisData rigidBodyData;
		rigidBodyData.mMOI = vehicleDesc.chassisMOI;
		rigidBodyData.mMass = vehicleDesc.chassisMass;
		rigidBodyData.mCMOffset = vehicleDesc.chassisCMOffset;

		veh4WActor = createVehicleActor
		(rigidBodyData,
			wheelConvexMeshes.data(), numWheels, wheelSimFilterData,
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
		physx::PxVec3 wheelCenterActorOffsets[PX_MAX_NB_WHEELS];
		const physx::PxF32 frontZ = chassisDims.z*0.3f;
		const physx::PxF32 rearZ = -chassisDims.z*0.3f;
		computeWheelCenterActorOffsets4W(frontZ, rearZ, chassisDims, wheelWidth, wheelRadius, numWheels, wheelCenterActorOffsets);

		//Set up the simulation data for all wheels.
		setupWheelsSimulationData
		(vehicleDesc.wheelMass, vehicleDesc.wheelMOI, wheelRadius, wheelWidth, 
			numWheels, wheelCenterActorOffsets,
			vehicleDesc.chassisCMOffset, vehicleDesc.chassisMass,
			wheelsSimData.get());
		Con::cerr<<"OFFSETS:"<<Con::endl;
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

static std::shared_ptr<pragma::physics::PhysXMaterial> gMaterial;
VehicleDesc initVehicleDesc()
{
	//Set up the chassis mass, dimensions, moment of inertia, and center of mass offset.
	//The moment of inertia is just the moment of inertia of a cuboid but modified for easier steering.
	//Center of mass offset is 0.65m above the base of the chassis and 0.25m towards the front.
	const physx::PxF32 chassisMass = 1500.0f;
	const physx::PxVec3 chassisDims(2.5f,2.0f,5.0f);
	const physx::PxVec3 chassisMOI
	((chassisDims.y*chassisDims.y + chassisDims.z*chassisDims.z)*chassisMass/12.0f,
		(chassisDims.x*chassisDims.x + chassisDims.z*chassisDims.z)*0.8f*chassisMass/12.0f,
		(chassisDims.x*chassisDims.x + chassisDims.y*chassisDims.y)*chassisMass/12.0f);
	const physx::PxVec3 chassisCMOffset(0.0f, -chassisDims.y*0.5f + 0.65f, 0.25f);

	//Set up the wheel mass, radius, width, moment of inertia, and number of wheels.
	//Moment of inertia is just the moment of inertia of a cylinder.
	const physx::PxF32 wheelMass = 20.0f;
	const physx::PxF32 wheelRadius = 0.5f;
	const physx::PxF32 wheelWidth = 0.4f;
	const physx::PxF32 wheelMOI = 0.5f*wheelMass*wheelRadius*wheelRadius;
	const physx::PxU32 nbWheels = 4;

	VehicleDesc vehicleDesc;

	vehicleDesc.chassisMass = chassisMass;
	vehicleDesc.chassisDims = chassisDims;
	vehicleDesc.chassisMOI = chassisMOI;
	vehicleDesc.chassisCMOffset = chassisCMOffset;
	vehicleDesc.chassisMaterial = &gMaterial->GetInternalObject();
	vehicleDesc.chassisSimFilterData = physx::PxFilterData(COLLISION_FLAG_CHASSIS, COLLISION_FLAG_CHASSIS_AGAINST, 0, 0);

	vehicleDesc.wheelMass = wheelMass;
	vehicleDesc.wheelRadius = wheelRadius;
	vehicleDesc.wheelWidth = wheelWidth;
	vehicleDesc.wheelMOI = wheelMOI;
	vehicleDesc.numWheels = nbWheels;
	vehicleDesc.wheelMaterial = &gMaterial->GetInternalObject();
	vehicleDesc.chassisSimFilterData = physx::PxFilterData(COLLISION_FLAG_WHEEL, COLLISION_FLAG_WHEEL_AGAINST, 0, 0);

	return vehicleDesc;
}

enum DriveMode
{
	eDRIVE_MODE_ACCEL_FORWARDS=0,
	eDRIVE_MODE_ACCEL_REVERSE,
	eDRIVE_MODE_HARD_TURN_LEFT,
	eDRIVE_MODE_HANDBRAKE_TURN_LEFT,
	eDRIVE_MODE_HARD_TURN_RIGHT,
	eDRIVE_MODE_HANDBRAKE_TURN_RIGHT,
	eDRIVE_MODE_BRAKE,
	eDRIVE_MODE_NONE
};

DriveMode gDriveModeOrder[] =
{
	eDRIVE_MODE_BRAKE,
	eDRIVE_MODE_ACCEL_FORWARDS,
	eDRIVE_MODE_BRAKE,
	eDRIVE_MODE_ACCEL_REVERSE,
	eDRIVE_MODE_BRAKE,
	eDRIVE_MODE_HARD_TURN_LEFT, 
	eDRIVE_MODE_BRAKE,
	eDRIVE_MODE_HARD_TURN_RIGHT,
	eDRIVE_MODE_ACCEL_FORWARDS,
	eDRIVE_MODE_HANDBRAKE_TURN_LEFT,
	eDRIVE_MODE_ACCEL_FORWARDS,
	eDRIVE_MODE_HANDBRAKE_TURN_RIGHT,
	eDRIVE_MODE_NONE
};
physx::PxVehicleDrive4WRawInputData gVehicleInputData;
physx::PxF32					gVehicleModeLifetime = 4.0f;
physx::PxF32					gVehicleModeTimer = 0.0f;
physx::PxU32					gVehicleOrderProgress = 0;
bool					gVehicleOrderComplete = false;
bool					gMimicKeyInputs = true;
physx::PxF32 gLengthScale = 1.f /0.025f;
snippetvehicle::VehicleSceneQueryData*	gVehicleSceneQueryData = NULL;
physx::PxBatchQuery*			gBatchQuery = NULL;

physx::PxVehicleDrivableSurfaceToTireFrictionPairs* gFrictionPairs = NULL;

util::TSharedHandle<pragma::physics::ICollisionObject>			gGroundPlane = nullptr;
physx::PxVehicleDrive4W*		gVehicle4W		= NULL;

bool					gIsVehicleInAir = true;
void startAccelerateForwardsMode()
{
	if(gMimicKeyInputs)
	{
		gVehicleInputData.setDigitalAccel(true);
	}
	else
	{
		gVehicleInputData.setAnalogAccel(1.0f);
	}
}

void startAccelerateReverseMode()
{
	gVehicle4W->mDriveDynData.forceGearChange(physx::PxVehicleGearsData::eREVERSE);

	if(gMimicKeyInputs)
	{
		gVehicleInputData.setDigitalAccel(true);
	}
	else
	{
		gVehicleInputData.setAnalogAccel(1.0f);
	}
}

void startBrakeMode()
{
	if(gMimicKeyInputs)
	{
		gVehicleInputData.setDigitalBrake(true);
	}
	else
	{
		gVehicleInputData.setAnalogBrake(1.0f);
	}
}

void startTurnHardLeftMode()
{
	if(gMimicKeyInputs)
	{
		gVehicleInputData.setDigitalAccel(true);
		gVehicleInputData.setDigitalSteerLeft(true);
	}
	else
	{
		gVehicleInputData.setAnalogAccel(true);
		gVehicleInputData.setAnalogSteer(-1.0f);
	}
}

void startTurnHardRightMode()
{
	if(gMimicKeyInputs)
	{
		gVehicleInputData.setDigitalAccel(true);
		gVehicleInputData.setDigitalSteerRight(true);
	}
	else
	{
		gVehicleInputData.setAnalogAccel(1.0f);
		gVehicleInputData.setAnalogSteer(1.0f);
	}
}

void startHandbrakeTurnLeftMode()
{
	if(gMimicKeyInputs)
	{
		gVehicleInputData.setDigitalSteerLeft(true);
		gVehicleInputData.setDigitalHandbrake(true);
	}
	else
	{
		gVehicleInputData.setAnalogSteer(-1.0f);
		gVehicleInputData.setAnalogHandbrake(1.0f);
	}
}

void startHandbrakeTurnRightMode()
{
	if(gMimicKeyInputs)
	{
		gVehicleInputData.setDigitalSteerRight(true);
		gVehicleInputData.setDigitalHandbrake(true);
	}
	else
	{
		gVehicleInputData.setAnalogSteer(1.0f);
		gVehicleInputData.setAnalogHandbrake(1.0f);
	}
}


void releaseAllControls()
{
	if(gMimicKeyInputs)
	{
		gVehicleInputData.setDigitalAccel(false);
		gVehicleInputData.setDigitalSteerLeft(false);
		gVehicleInputData.setDigitalSteerRight(false);
		gVehicleInputData.setDigitalBrake(false);
		gVehicleInputData.setDigitalHandbrake(false);
	}
	else
	{
		gVehicleInputData.setAnalogAccel(0.0f);
		gVehicleInputData.setAnalogSteer(0.0f);
		gVehicleInputData.setAnalogBrake(0.0f);
		gVehicleInputData.setAnalogHandbrake(0.0f);
	}
}

util::TSharedHandle<pragma::physics::ICollisionObject> createDrivablePlane(pragma::physics::PhysXEnvironment &env,const physx::PxFilterData& simFilterData, pragma::physics::PhysXMaterial &mat, physx::PxPhysics* physics)
{
	auto plane = env.CreatePlane(Vector3{0.f,1.f,0.f},0.f,mat);
	//Set the query filter data of the ground plane so that the vehicle raycasts can hit the ground.
	physx::PxFilterData qryFilterData;
	snippetvehicle::setupDrivableSurface(qryFilterData);

	auto &actorShape = *static_cast<pragma::physics::PhysXRigidStatic&>(pragma::physics::PhysXCollisionObject::GetCollisionObject(*plane)).GetActorShapeCollection().GetActorShapes().front();
	actorShape.GetActorShape().setQueryFilterData(qryFilterData);

	//Set the simulation filter data of the ground plane so that it collides with the chassis of a vehicle but not the wheels.
	actorShape.GetActorShape().setSimulationFilterData(simFilterData);

	plane->Spawn();
	return plane;
}

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

void incrementDrivingMode(const physx::PxF32 timestep)
{
	gVehicleModeTimer += timestep;
	if(gVehicleModeTimer > gVehicleModeLifetime)
	{
		//If the mode just completed was eDRIVE_MODE_ACCEL_REVERSE then switch back to forward gears.
		if(eDRIVE_MODE_ACCEL_REVERSE == gDriveModeOrder[gVehicleOrderProgress])
		{
			gVehicle4W->mDriveDynData.forceGearChange(physx::PxVehicleGearsData::eFIRST);
		}

		//Increment to next driving mode.
		gVehicleModeTimer = 0.0f;
		gVehicleOrderProgress++;
		releaseAllControls();

		//If we are at the end of the list of driving modes then start again.
		if(eDRIVE_MODE_NONE == gDriveModeOrder[gVehicleOrderProgress])
		{
			gVehicleOrderProgress = 0;
			gVehicleOrderComplete = true;
		}

		//Start driving in the selected mode.
		DriveMode eDriveMode = gDriveModeOrder[gVehicleOrderProgress];
		switch(eDriveMode)
		{
		case eDRIVE_MODE_ACCEL_FORWARDS:
			startAccelerateForwardsMode();
			break;
		case eDRIVE_MODE_ACCEL_REVERSE:
			startAccelerateReverseMode();
			break;
		case eDRIVE_MODE_HARD_TURN_LEFT:
			startTurnHardLeftMode();
			break;
		case eDRIVE_MODE_HANDBRAKE_TURN_LEFT:
			startHandbrakeTurnLeftMode();
			break;
		case eDRIVE_MODE_HARD_TURN_RIGHT:
			startTurnHardRightMode();
			break;
		case eDRIVE_MODE_HANDBRAKE_TURN_RIGHT:
			startHandbrakeTurnRightMode();
			break;
		case eDRIVE_MODE_BRAKE:
			startBrakeMode();
			break;
		case eDRIVE_MODE_NONE:
			break;
		};

		//If the mode about to start is eDRIVE_MODE_ACCEL_REVERSE then switch to reverse gears.
		if(eDRIVE_MODE_ACCEL_REVERSE == gDriveModeOrder[gVehicleOrderProgress])
		{
			gVehicle4W->mDriveDynData.forceGearChange(physx::PxVehicleGearsData::eREVERSE);
		}
	}
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
util::TSharedHandle<pragma::physics::IVehicle> pragma::physics::PhysXEnvironment::CreateVehicle()
{
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

	gMaterial = std::dynamic_pointer_cast<PhysXMaterial>(CreateMaterial(0.5f,0.5f,0.6f));

	///

	//Create the batched scene queries for the suspension raycasts.
	gVehicleSceneQueryData = snippetvehicle::VehicleSceneQueryData::allocate(1, PX_MAX_NB_WHEELS, 1, 1, snippetvehicle::WheelSceneQueryPreFilterBlocking, NULL);
	gBatchQuery = snippetvehicle::VehicleSceneQueryData::setUpBatchedSceneQuery(0, *gVehicleSceneQueryData, gScene);

	//Create the friction table for each combination of tire and surface type.
	gFrictionPairs = createFrictionPairs(&gMaterial->GetInternalObject(),&PhysXMaterial::GetMaterial(GetGenericMaterial()).GetInternalObject());

	//Create a plane to drive on.
	physx::PxFilterData groundPlaneSimFilterData(COLLISION_FLAG_GROUND, COLLISION_FLAG_GROUND_AGAINST, 0, 0);
	gGroundPlane = createDrivablePlane(*this,groundPlaneSimFilterData,*gMaterial, gPhysics);

	//Create a vehicle that will drive on the plane.
	VehicleDesc vehicleDesc = initVehicleDesc();
	util::TSharedHandle<pragma::physics::PhysXRigidBody> rigidBody;
	auto gVehicle = px_create_unique_ptr<physx::PxVehicleDrive>(createVehicle4W(*this,vehicleDesc, gPhysics, gCooking,rigidBody));
	gVehicle4W = static_cast<physx::PxVehicleDrive4W*>(gVehicle.get());


	//Convert the vehicle from meters to the chosen length scale.
	customizeVehicleToLengthScale(gLengthScale, gVehicle4W->getRigidDynamicActor(), &gVehicle4W->mWheelsSimData, &gVehicle4W->mDriveSimData);

	//Convert the steer angle vs forward speed table to the chosen length scale.
	for(physx::PxU32 i = 0; i < gSteerVsForwardSpeedTable.mNbDataPairs; i++)
	{
		gSteerVsForwardSpeedTable.mDataPairs[2*i +0] *= gLengthScale;
	}

	InitializeCollisionObject(*rigidBody);

	physx::PxTransform startTransform(physx::PxVec3(0, ((vehicleDesc.chassisDims.y*0.5f + vehicleDesc.wheelRadius + 1.0f)*gLengthScale), 0), physx::PxQuat(physx::PxIdentity));
	gVehicle4W->getRigidDynamicActor()->setGlobalPose(startTransform);

	//Set the vehicle to rest in first gear.
	//Set the vehicle to use auto-gears.
	gVehicle4W->setToRestState();
	gVehicle4W->mDriveDynData.forceGearChange(snippetvehicle::PxVehicleGearsData::eFIRST);
	gVehicle4W->mDriveDynData.setUseAutoGears(true);

	gVehicleModeTimer = 0.0f;
	gVehicleOrderProgress = 0;
	startBrakeMode();

	auto vhc = CreateSharedHandle<PhysXVehicle>(*this,std::move(gVehicle),util::shared_handle_cast<PhysXRigidBody,ICollisionObject>(rigidBody));
	AddVehicle(*vhc);
	vhc->Spawn();
	return util::shared_handle_cast<PhysXVehicle,IVehicle>(vhc);
}
#pragma optimize("",on)
