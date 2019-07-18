#include "pr_physx/environment.hpp"
#include "pr_physx/vehicle.hpp"
#include "pr_physx/material.hpp"
#include "pr_physx/collision_object.hpp"
#include "pr_physx/shape.hpp"
#include "pr_physx/query_filter_callback.hpp"
#include <pragma/networkstate/networkstate.h>
#include <pragma/util/util_game.hpp>
#include <vehicle/PxVehicleSDK.h>

#pragma optimize("",off)
pragma::physics::VehicleSceneQueryData::VehicleSceneQueryData(uint32_t numQueriesInBatch)
	: m_batchQueryDesc{numQueriesInBatch,numQueriesInBatch,0}
{}

std::unique_ptr<pragma::physics::VehicleSceneQueryData> pragma::physics::VehicleSceneQueryData::Create(pragma::physics::PhysXEnvironment &env,uint32_t numWheels,physx::PxBatchQueryPreFilterShader preFilterShader,physx::PxBatchQueryPostFilterShader postFilterShader)
{
	auto numHitPoints = numWheels *1;
	auto numQueriesPerBatch = 1*numWheels;
	auto sqData = std::make_unique<VehicleSceneQueryData>(numQueriesPerBatch);
	sqData->m_numQueriesPerBatch = numQueriesPerBatch;
	sqData->m_numHitResultsPerQuery = numHitPoints;

	sqData->m_raycastResults.resize(numWheels);
	sqData->m_raycastHitBuffer.resize(numHitPoints);
	sqData->m_sweepResults.resize(numWheels);
	sqData->m_sweepHitBuffer.resize(numHitPoints);

	sqData->m_preFilterShader = preFilterShader;
	sqData->m_postFilterShader = postFilterShader;

	// Batch query
	const physx::PxU32 maxNumQueriesInBatch =  sqData->m_numQueriesPerBatch;
	const physx::PxU32 maxNumHitResultsInBatch = sqData->m_numQueriesPerBatch*sqData->m_numHitResultsPerQuery;

	sqData->m_batchQueryDesc.queryMemory.userRaycastResultBuffer = const_cast<physx::PxRaycastQueryResult*>(sqData->m_raycastResults.data());
	sqData->m_batchQueryDesc.queryMemory.userRaycastTouchBuffer = const_cast<physx::PxRaycastHit*>(sqData->m_raycastHitBuffer.data());
	sqData->m_batchQueryDesc.queryMemory.raycastTouchBufferSize = maxNumHitResultsInBatch;

	sqData->m_batchQueryDesc.queryMemory.userSweepResultBuffer = const_cast<physx::PxSweepQueryResult*>(sqData->m_sweepResults.data());
	sqData->m_batchQueryDesc.queryMemory.userSweepTouchBuffer = const_cast<physx::PxSweepHit*>(sqData->m_sweepHitBuffer.data());
	sqData->m_batchQueryDesc.queryMemory.sweepTouchBufferSize = maxNumHitResultsInBatch;

	sqData->m_batchQueryDesc.preFilterShader = sqData->m_preFilterShader;
	sqData->m_batchQueryDesc.postFilterShader = sqData->m_postFilterShader;

	sqData->m_batchQuery = pragma::physics::px_create_unique_ptr(env.GetScene().createBatchQuery(sqData->m_batchQueryDesc));
	return sqData;
}

const std::vector<physx::PxRaycastQueryResult> &pragma::physics::VehicleSceneQueryData::GetRaycastResults() const {return const_cast<VehicleSceneQueryData*>(this)->GetRaycastResults();}
std::vector<physx::PxRaycastQueryResult> &pragma::physics::VehicleSceneQueryData::GetRaycastResults() {return m_raycastResults;}
const std::vector<physx::PxSweepQueryResult> &pragma::physics::VehicleSceneQueryData::GetSweepResults() const {return const_cast<VehicleSceneQueryData*>(this)->GetSweepResults();}
std::vector<physx::PxSweepQueryResult> &pragma::physics::VehicleSceneQueryData::GetSweepResults() {return m_sweepResults;}
const std::vector<physx::PxRaycastHit> &pragma::physics::VehicleSceneQueryData::GetRaycastHits() const {return const_cast<VehicleSceneQueryData*>(this)->GetRaycastHits();}
std::vector<physx::PxRaycastHit> &pragma::physics::VehicleSceneQueryData::GetRaycastHits() {return m_raycastHitBuffer;}
const std::vector<physx::PxSweepHit> &pragma::physics::VehicleSceneQueryData::GetSweepHits() const {return const_cast<VehicleSceneQueryData*>(this)->GetSweepHits();}
std::vector<physx::PxSweepHit> &pragma::physics::VehicleSceneQueryData::GetSweepHits() {return m_sweepHitBuffer;}

physx::PxBatchQuery &pragma::physics::VehicleSceneQueryData::GetBatchQuery() {return *m_batchQuery;}

const physx::PxBatchQueryPreFilterShader &pragma::physics::VehicleSceneQueryData::GetPreFilterShader() const {return m_preFilterShader;}
const physx::PxBatchQueryPostFilterShader &pragma::physics::VehicleSceneQueryData::GetPostFilterShader() const {return m_postFilterShader;}

physx::PxU32 pragma::physics::VehicleSceneQueryData::GetNumQueriesPerBatch() const {return m_numQueriesPerBatch;}
physx::PxU32 pragma::physics::VehicleSceneQueryData::GetNumHitResultsPerQuery() const {return m_numHitResultsPerQuery;}

static void customizeVehicleToLengthScale(
	physx::PxReal lengthScale,physx::PxRigidDynamic &rigidDynamic,physx::PxVehicleWheelsSimData &wheelsSimData,physx::PxVehicleDriveSimData &driveSimData
)
{
	//Rigid body center of mass and moment of inertia.
	{
		physx::PxVec3 moi = rigidDynamic.getMassSpaceInertiaTensor();
		moi *= (lengthScale*lengthScale);
		rigidDynamic.setMassSpaceInertiaTensor(moi);
	}

	//Wheels, suspensions, wheel centers, tire/susp force application points.
	{
		for(physx::PxU32 i = 0; i < wheelsSimData.getNbWheels(); i++)
		{
			physx::PxVehicleWheelData wheelData = wheelsSimData.getWheelData(i);
			wheelData.mRadius *= lengthScale;
			wheelData.mWidth *= lengthScale;
			wheelData.mDampingRate *= lengthScale*lengthScale;
			wheelData.mMaxBrakeTorque *= lengthScale*lengthScale;
			wheelData.mMaxHandBrakeTorque *= lengthScale*lengthScale;
			wheelData.mMOI *= lengthScale*lengthScale;
			wheelsSimData.setWheelData(i, wheelData);

			physx::PxVehicleSuspensionData suspData = wheelsSimData.getSuspensionData(i);
			suspData.mMaxCompression *= lengthScale;
			suspData.mMaxDroop *= lengthScale;
			wheelsSimData.setSuspensionData(i, suspData);

			physx::PxVec3 v = wheelsSimData.getWheelCentreOffset(i);
			v *= lengthScale;
			wheelsSimData.setWheelCentreOffset(i,v);

			v = wheelsSimData.getSuspForceAppPointOffset(i);
			v *= lengthScale;
			wheelsSimData.setSuspForceAppPointOffset(i,v);

			v = wheelsSimData.getTireForceAppPointOffset(i);
			v *= lengthScale;
			wheelsSimData.setTireForceAppPointOffset(i,v);
		}
	}

	//Slow forward speed correction.
	{
		wheelsSimData.setSubStepCount(5.0f*lengthScale, 3, 1);
		wheelsSimData.setMinLongSlipDenominator(4.0f*lengthScale);
	}

	//Engine
	physx::PxVehicleEngineData engineData = driveSimData.getEngineData();
	engineData.mMOI *= lengthScale*lengthScale;
	engineData.mPeakTorque *= lengthScale*lengthScale;
	engineData.mDampingRateFullThrottle *= lengthScale*lengthScale;
	engineData.mDampingRateZeroThrottleClutchEngaged *= lengthScale*lengthScale;
	engineData.mDampingRateZeroThrottleClutchDisengaged *= lengthScale*lengthScale;
	driveSimData.setEngineData(engineData);

	//Clutch
	physx::PxVehicleClutchData clutchData = driveSimData.getClutchData();
	clutchData.mStrength *= lengthScale*lengthScale;
	driveSimData.setClutchData(clutchData);
}

static void customizeVehicleToLengthScale(
	physx::PxReal lengthScale,physx::PxRigidDynamic &rigidDynamic,physx::PxVehicleWheelsSimData &wheelsSimData,physx::PxVehicleDriveSimData4W &driveSimData
)
{
	customizeVehicleToLengthScale(lengthScale,rigidDynamic,wheelsSimData,static_cast<physx::PxVehicleDriveSimData&>(driveSimData));

	//Ackermann geometry.
	physx::PxVehicleAckermannGeometryData ackermannData = driveSimData.getAckermannGeometryData();
	ackermannData.mAxleSeparation *= lengthScale;
	ackermannData.mFrontWidth *= lengthScale;
	ackermannData.mRearWidth *= lengthScale;
	driveSimData.setAckermannGeometryData(ackermannData);
}

static util::TSharedHandle<pragma::physics::PhysXRigidBody> createVehicleActor(
	const physx::PxVehicleChassisData &chassisData,
	const pragma::physics::VehicleCreateInfo &vhcDesc
)
{
	auto &rigidBody = pragma::physics::PhysXCollisionObject::GetCollisionObject(*vhcDesc.actor);
	auto &dynamicRigidActor = static_cast<physx::PxRigidDynamic&>(rigidBody.GetInternalObject());
	dynamicRigidActor.setMassSpaceInertiaTensor(chassisData.mMOI);
	return util::shared_handle_cast<pragma::physics::IBase,pragma::physics::PhysXRigidBody>(rigidBody.ClaimOwnership());
}

static void setupWheelsSimulationData(
	pragma::physics::PhysXEnvironment &env,const pragma::physics::VehicleCreateInfo &vhcCreateInfo,
	const physx::PxVec3 &chassisCMOffset,const physx::PxF32 chassisMass,
	physx::PxVehicleWheelsSimData &outWheelsSimData,
	pragma::physics::NoCollisionCategoryId colCategory
)
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
	constexpr auto scale = util::metres_to_units(1.f);
	std::vector<WheelData> wheels {};
	wheels.reserve(vhcCreateInfo.wheels.size());
	std::vector<physx::PxVec3> wheelCenterActorOffsets;
	for(auto &createInfo : vhcCreateInfo.wheels)
	{
		wheels.push_back({});
		auto &wheel = wheels.back();

		wheel.data.mMass = 0.f;
		auto &actorShapes = pragma::physics::PhysXCollisionObject::GetCollisionObject(*vhcCreateInfo.actor).GetActorShapeCollection().GetActorShapes();
		if(createInfo.shapeIndex >= 0 && createInfo.shapeIndex < actorShapes.size())
		{
			auto &actorShape = actorShapes.at(createInfo.shapeIndex);
			wheel.data.mMass = actorShape->GetShape().GetMass();
		}
		wheel.data.mMOI = createInfo.GetMomentOfInertia(*vhcCreateInfo.actor) /umath::pow2(scale);

		wheel.data.mRadius = createInfo.radius /scale;
		wheel.data.mWidth = createInfo.width /scale;
		wheel.data.mMaxHandBrakeTorque = createInfo.maxHandbrakeTorque /umath::pow2(scale);
		wheel.data.mMaxSteer = umath::deg_to_rad(createInfo.maxSteeringAngle);
		wheel.tire.mType = createInfo.tireType;

		wheelCenterActorOffsets.push_back(env.ToPhysXVector(createInfo.chassisOffset) /scale);
	}

	std::vector<physx::PxF32> suspSprungMasses(vhcCreateInfo.wheels.size());
	physx::PxVehicleComputeSprungMasses(
		vhcCreateInfo.wheels.size(),wheelCenterActorOffsets.data(), 
		chassisCMOffset,chassisMass,1 /* gravity direction relative to chassis */,suspSprungMasses.data()
	);

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

	// Only collision with drivable surfaces
	physx::PxFilterData qryFilterData {umath::to_integral(CollisionMask::Vehicle),umath::to_integral(CollisionMask::All),0,colCategory};

	wheelIdx = 0u;
	for(auto &wheel : wheels)
	{
		auto &wheelCreateInfo = vhcCreateInfo.wheels.at(wheelIdx);
		outWheelsSimData.setWheelData(wheelIdx,wheel.data);
		outWheelsSimData.setTireData(wheelIdx,wheel.tire);
		outWheelsSimData.setSuspensionData(wheelIdx,wheel.suspension);
		outWheelsSimData.setSuspTravelDirection(wheelIdx,wheel.suspTravelDirection);
		outWheelsSimData.setWheelCentreOffset(wheelIdx,wheel.wheelCentreCMOffset);
		outWheelsSimData.setSuspForceAppPointOffset(wheelIdx,wheel.suspForceAppCMOffset);
		outWheelsSimData.setTireForceAppPointOffset(wheelIdx,wheel.tireForceAppCMOffset);
		outWheelsSimData.setSceneQueryFilterData(wheelIdx,qryFilterData);
		outWheelsSimData.setWheelShapeMapping(wheelIdx,wheelCreateInfo.shapeIndex);
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
		outWheelsSimData.addAntiRollBarData(bar);
	}
}

static pragma::physics::PhysXUniquePtr<physx::PxVehicleDrive> createVehicle4W(
	pragma::physics::PhysXEnvironment &env,
	const pragma::physics::VehicleCreateInfo &vhcCreateInfo,
	util::TSharedHandle<pragma::physics::PhysXRigidBody> &outRigidBody,
	pragma::physics::NoCollisionCategoryId colCategory
)
{
	const physx::PxU32 numWheels = vhcCreateInfo.wheels.size();

	constexpr auto scale = util::metres_to_units(1.f);

	//Construct a physx actor with shapes for the chassis and wheels.
	//Set the rigid body mass, moment of inertia, and center of mass offset.
	auto cmOffset = env.ToPhysXVector(vhcCreateInfo.actor->GetCenterOfMassOffset()) /scale;
	util::TSharedHandle<pragma::physics::PhysXRigidBody> veh4WActor = nullptr;
	{
		//Rigid body data.
		physx::PxVehicleChassisData rigidBodyData;
		rigidBodyData.mMOI = env.ToPhysXVector(vhcCreateInfo.chassis.GetMomentOfInertia(*vhcCreateInfo.actor)) /umath::pow2(scale);
		rigidBodyData.mCMOffset = cmOffset;

		veh4WActor = createVehicleActor(rigidBodyData,vhcCreateInfo);
	}

	//Set up the sim data for the wheels.
	std::unique_ptr<physx::PxVehicleWheelsSimData,void(*)(physx::PxVehicleWheelsSimData*)> wheelsSimData {
		physx::PxVehicleWheelsSimData::allocate(numWheels),
		[](physx::PxVehicleWheelsSimData *simData) {
			simData->free();
		}
	};

	auto mass = 0.f;
	auto &actorShapes = pragma::physics::PhysXCollisionObject::GetCollisionObject(*vhcCreateInfo.actor).GetActorShapeCollection().GetActorShapes();
	if(vhcCreateInfo.chassis.shapeIndex >= 0 && vhcCreateInfo.chassis.shapeIndex < actorShapes.size())
	{
		auto &actorShape = actorShapes.at(vhcCreateInfo.chassis.shapeIndex);
		mass = actorShape->GetShape().GetMass();
	}
	// The mass of the actor has to match the mass of the chassis,
	// we'll just enforce that here
	const_cast<pragma::physics::IRigidBody&>(*vhcCreateInfo.actor).SetMass(mass);

	//Set up the simulation data for all wheels.
	setupWheelsSimulationData(
		env,vhcCreateInfo,
		cmOffset,
		mass,
		*wheelsSimData,
		colCategory
	);

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
		engine.mPeakTorque = vhcCreateInfo.maxEngineTorque;
		engine.mMaxOmega = vhcCreateInfo.maxEngineRotationSpeed;
		driveSimData.setEngineData(engine);

		//Gears
		physx::PxVehicleGearsData gears;
		gears.mSwitchTime = vhcCreateInfo.gearSwitchTime;
		driveSimData.setGearsData(gears);

		//Clutch
		physx::PxVehicleClutchData clutch;
		clutch.mStrength = vhcCreateInfo.clutchStrength;
		driveSimData.setClutchData(clutch);

		//Ackermann steer accuracy
		physx::PxVehicleAckermannGeometryData ackermann;
		ackermann.mAccuracy = 1.0f;
		ackermann.mAxleSeparation =
			wheelsSimData->getWheelCentreOffset(physx::PxVehicleDrive4WWheelOrder::eFRONT_LEFT).z-
			wheelsSimData->getWheelCentreOffset(physx::PxVehicleDrive4WWheelOrder::eREAR_LEFT).z;
		ackermann.mFrontWidth =
			wheelsSimData->getWheelCentreOffset(physx::PxVehicleDrive4WWheelOrder::eFRONT_RIGHT).x-
			wheelsSimData->getWheelCentreOffset(physx::PxVehicleDrive4WWheelOrder::eFRONT_LEFT).x;
		ackermann.mRearWidth =
			wheelsSimData->getWheelCentreOffset(physx::PxVehicleDrive4WWheelOrder::eREAR_RIGHT).x -
			wheelsSimData->getWheelCentreOffset(physx::PxVehicleDrive4WWheelOrder::eREAR_LEFT).x;
		driveSimData.setAckermannGeometryData(ackermann);
	}

	//Create a vehicle from the wheels and drive sim data.
	auto *vehDrive4W = physx::PxVehicleDrive4W::allocate(numWheels);
	vehDrive4W->setup(&env.GetPhysics(),&static_cast<physx::PxRigidDynamic&>(veh4WActor->GetInternalObject()),*wheelsSimData,driveSimData,numWheels -4);
	//Configure the userdata
	outRigidBody = veh4WActor;
	return pragma::physics::px_create_unique_ptr<physx::PxVehicleDrive>(vehDrive4W);
}

util::TSharedHandle<pragma::physics::IVehicle> pragma::physics::PhysXEnvironment::CreateVehicle(const VehicleCreateInfo &vhcDesc)
{
	if(vhcDesc.actor.IsExpired())
		return nullptr;

	// Collision groups and masks have to be changed for the actor

	// The vehicle actor must not collide with itself
	auto colCategory = pragma::physics::PhysXCollisionObject::GetCollisionObject(*vhcDesc.actor).DisableSelfCollisions();

	auto &actorShapes = pragma::physics::PhysXCollisionObject::GetCollisionObject(*vhcDesc.actor).GetActorShapeCollection().GetActorShapes();
	for(auto &wheel : vhcDesc.wheels)
	{
		auto shapeIndex = wheel.shapeIndex;
		if(shapeIndex < 0 || shapeIndex >= actorShapes.size())
			continue;
		auto &actorShape = actorShapes.at(shapeIndex)->GetActorShape();
		// Disable simulation collision for the wheel
		actorShape.setSimulationFilterData({umath::to_integral(CollisionMask::NoCollision),0,0,colCategory});
	}

	//Create the batched scene queries for the suspension raycasts.
	auto sceneQueryData = pragma::physics::VehicleSceneQueryData::Create(*this,vhcDesc.wheels.size(),pragma::physics::BatchQueryPreFilterBlocking,nullptr);
	if(sceneQueryData == nullptr)
		return nullptr;

	//Create a vehicle that will drive on the plane.
	util::TSharedHandle<pragma::physics::PhysXRigidBody> rigidBody;
	auto gVehicle = createVehicle4W(*this,vhcDesc,rigidBody,colCategory);
	auto *gVehicle4W = static_cast<physx::PxVehicleDrive4W*>(gVehicle.get());

	//Convert the vehicle from meters to the chosen length scale.
	constexpr auto scale = util::metres_to_units(1.f);
	customizeVehicleToLengthScale(scale,*gVehicle4W->getRigidDynamicActor(),gVehicle4W->mWheelsSimData,gVehicle4W->mDriveSimData);

	//Convert the steer angle vs forward speed table to the chosen length scale.
	static std::array<physx::PxF32,2 *4> steerVsForwardSpeedData = {
		0.0f,		0.75f,
		5.0f,		0.75f,
		30.0f,		0.125f,
		120.0f,		0.1f
	};
	physx::PxFixedSizeLookupTable<8> steerVsForwardSpeedTable {steerVsForwardSpeedData.data(),steerVsForwardSpeedData.size() /2};
	for(auto i=decltype(steerVsForwardSpeedTable.mNbDataPairs){0u};i<steerVsForwardSpeedTable.mNbDataPairs;++i)
		steerVsForwardSpeedTable.mDataPairs[2 *i] *= scale;

	InitializeCollisionObject(*rigidBody);

	auto vhc = CreateSharedHandle<PhysXVehicle>(
		*this,std::move(gVehicle),util::shared_handle_cast<PhysXRigidBody,ICollisionObject>(rigidBody),
		std::move(sceneQueryData),steerVsForwardSpeedTable
	);
	AddVehicle(*vhc);
	vhc->Spawn();
	vhc->SetRestState();
	return util::shared_handle_cast<PhysXVehicle,IVehicle>(vhc);
}
#pragma optimize("",on)
