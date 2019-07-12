#include "pr_physx/environment.hpp"
#include "pr_physx/vehicle.hpp"
#include "pr_physx/material.hpp"
#include "pr_physx/collision_object.hpp"
#include "pr_physx/shape.hpp"
#include <pragma/networkstate/networkstate.h>
#include <vehicle/PxVehicleSDK.h>

enum
{
	TIRE_TYPE_NORMAL=0,
	TIRE_TYPE_WORN,
	MAX_NUM_TIRE_TYPES
};

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

static physx::PxRigidDynamic* setupVehicleActor
(const physx::PxVehicleChassisData& chassisData,
	physx::PxMaterial** wheelMaterials, physx::PxConvexMesh** wheelConvexMeshes, const physx::PxU32 numWheels, const physx::PxFilterData& wheelSimFilterData,
	physx::PxMaterial** chassisMaterials, physx::PxConvexMesh** chassisConvexMeshes, const physx::PxU32 numChassisMeshes, const physx::PxFilterData& chassisSimFilterData,
	physx::PxPhysics& physics)
{
	//We need a rigid body actor for the vehicle.
	//Don't forget to add the actor to the scene after setting up the associated vehicle.
	physx::PxRigidDynamic* vehActor = physics.createRigidDynamic(physx::PxTransform(physx::PxIdentity));

	//Wheel and chassis query filter data.
	//Optional: cars don't drive on other cars.
	physx::PxFilterData wheelQryFilterData;
	setupNonDrivableSurface(wheelQryFilterData);
	physx::PxFilterData chassisQryFilterData;
	setupNonDrivableSurface(chassisQryFilterData);

	//Add all the wheel shapes to the actor.
	for(physx::PxU32 i = 0; i < numWheels; i++)
	{
		physx::PxConvexMeshGeometry geom(wheelConvexMeshes[i]);
		physx::PxShape* wheelShape=physx::PxRigidActorExt::createExclusiveShape(*vehActor, geom, *wheelMaterials[i]);
		wheelShape->setQueryFilterData(wheelQryFilterData);
		wheelShape->setSimulationFilterData(wheelSimFilterData);
		wheelShape->setLocalPose(physx::PxTransform(physx::PxIdentity));
	}

	//Add the chassis shapes to the actor.
	for(physx::PxU32 i = 0; i < numChassisMeshes; i++)
	{
		physx::PxShape* chassisShape=physx::PxRigidActorExt::createExclusiveShape(*vehActor, physx::PxConvexMeshGeometry(chassisConvexMeshes[i]), *chassisMaterials[i]);
		chassisShape->setQueryFilterData(chassisQryFilterData);
		chassisShape->setSimulationFilterData(chassisSimFilterData);
		chassisShape->setLocalPose(physx::PxTransform(physx::PxIdentity));
	}

	vehActor->setMass(chassisData.mMass);
	vehActor->setMassSpaceInertiaTensor(chassisData.mMOI);
	vehActor->setCMassLocalPose(physx::PxTransform(chassisData.mCMOffset,physx::PxQuat(physx::PxIdentity)));

	return vehActor;
}

static physx::PxConvexMesh* createConvexMesh(const physx::PxVec3* verts, const physx::PxU32 numVerts, physx::PxPhysics& physics, physx::PxCooking& cooking)
{
	// Create descriptor for convex mesh
	physx::PxConvexMeshDesc convexDesc;
	convexDesc.points.count			= numVerts;
	convexDesc.points.stride		= sizeof(physx::PxVec3);
	convexDesc.points.data			= verts;
	convexDesc.flags				= physx::PxConvexFlag::eCOMPUTE_CONVEX;

	physx::PxConvexMesh* convexMesh = NULL;
	physx::PxDefaultMemoryOutputStream buf;
	if(cooking.cookConvexMesh(convexDesc, buf))
	{
		physx::PxDefaultMemoryInputData id(buf.getData(), buf.getSize());
		convexMesh = physics.createConvexMesh(id);
	}

	return convexMesh;
}

static float scale = 1.f; // 1.f /0.025f;
physx::PxConvexMesh* createChassisMesh(const physx::PxVec3 dims, physx::PxPhysics& physics, physx::PxCooking& cooking)
{
	const physx::PxF32 x = dims.x*0.5f *scale;
	const physx::PxF32 y = dims.y*0.5f *scale;
	const physx::PxF32 z = dims.z*0.5f *scale;
	physx::PxVec3 verts[8] =
	{
		physx::PxVec3(x,y,-z), 
		physx::PxVec3(x,y,z),
		physx::PxVec3(x,-y,z),
		physx::PxVec3(x,-y,-z),
		physx::PxVec3(-x,y,-z), 
		physx::PxVec3(-x,y,z),
		physx::PxVec3(-x,-y,z),
		physx::PxVec3(-x,-y,-z)
	};

	return createConvexMesh(verts,8,physics,cooking);
}

physx::PxConvexMesh* createWheelMesh(const physx::PxF32 width, const physx::PxF32 radius, physx::PxPhysics& physics, physx::PxCooking& cooking)
{
	physx::PxVec3 points[2*16];
	for(physx::PxU32 i = 0; i < 16; i++)
	{
		const physx::PxF32 cosTheta = physx::PxCos(i*physx::PxPi*2.0f/16.0f);
		const physx::PxF32 sinTheta = physx::PxSin(i*physx::PxPi*2.0f/16.0f);
		const physx::PxF32 y = radius*cosTheta;
		const physx::PxF32 z = radius*sinTheta;
		points[2*i+0] = physx::PxVec3(-width/2.0f, y, z) *scale;
		points[2*i+1] = physx::PxVec3(+width/2.0f, y, z) *scale;
	}

	return createConvexMesh(points,32,physics,cooking);
}

physx::PxRigidDynamic* createVehicleActor
(const physx::PxVehicleChassisData& chassisData,
	physx::PxMaterial** wheelMaterials, physx::PxConvexMesh** wheelConvexMeshes, const physx::PxU32 numWheels, const physx::PxFilterData& wheelSimFilterData,
	physx::PxMaterial** chassisMaterials, physx::PxConvexMesh** chassisConvexMeshes, const physx::PxU32 numChassisMeshes, const physx::PxFilterData& chassisSimFilterData,
	physx::PxPhysics& physics)
{
	//We need a rigid body actor for the vehicle.
	//Don't forget to add the actor to the scene after setting up the associated vehicle.
	physx::PxRigidDynamic* vehActor = physics.createRigidDynamic(physx::PxTransform(physx::PxIdentity));

	//Wheel and chassis query filter data.
	//Optional: cars don't drive on other cars.
	physx::PxFilterData wheelQryFilterData;
	setupNonDrivableSurface(wheelQryFilterData);
	physx::PxFilterData chassisQryFilterData;
	setupNonDrivableSurface(chassisQryFilterData);

	//Add all the wheel shapes to the actor.
	for(physx::PxU32 i = 0; i < numWheels; i++)
	{
		physx::PxConvexMeshGeometry geom(wheelConvexMeshes[i]);
		physx::PxShape* wheelShape=physx::PxRigidActorExt::createExclusiveShape(*vehActor, geom, *wheelMaterials[i]);
		wheelShape->setQueryFilterData(wheelQryFilterData);
		wheelShape->setSimulationFilterData(wheelSimFilterData);
		wheelShape->setLocalPose(physx::PxTransform(physx::PxIdentity));
	}

	//Add the chassis shapes to the actor.
	for(physx::PxU32 i = 0; i < numChassisMeshes; i++)
	{
		physx::PxShape* chassisShape=physx::PxRigidActorExt::createExclusiveShape(*vehActor, physx::PxConvexMeshGeometry(chassisConvexMeshes[i]), *chassisMaterials[i]);
		chassisShape->setQueryFilterData(chassisQryFilterData);
		chassisShape->setSimulationFilterData(chassisSimFilterData);
		chassisShape->setLocalPose(physx::PxTransform(physx::PxIdentity));
	}

	vehActor->setMass(chassisData.mMass);
	vehActor->setMassSpaceInertiaTensor(chassisData.mMOI);
	vehActor->setCMassLocalPose(physx::PxTransform(chassisData.mCMOffset,physx::PxQuat(physx::PxIdentity)));

	return vehActor;
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
	wheelCentreOffsets[physx::PxVehicleDrive4WWheelOrder::eREAR_LEFT] = physx::PxVec3((-chassisDims.x + wheelWidth)*0.5f, -(chassisDims.y/2 + wheelRadius), wheelRearZ + 0*deltaZ*0.5f) *scale;
	wheelCentreOffsets[physx::PxVehicleDrive4WWheelOrder::eREAR_RIGHT] = physx::PxVec3((+chassisDims.x - wheelWidth)*0.5f, -(chassisDims.y/2 + wheelRadius), wheelRearZ + 0*deltaZ*0.5f) *scale;
	wheelCentreOffsets[physx::PxVehicleDrive4WWheelOrder::eFRONT_LEFT] = physx::PxVec3((-chassisDims.x + wheelWidth)*0.5f, -(chassisDims.y/2 + wheelRadius), wheelRearZ + (numLeftWheels-1)*deltaZ) *scale;
	wheelCentreOffsets[physx::PxVehicleDrive4WWheelOrder::eFRONT_RIGHT] = physx::PxVec3((+chassisDims.x - wheelWidth)*0.5f, -(chassisDims.y/2 + wheelRadius), wheelRearZ + (numLeftWheels-1)*deltaZ) *scale;
	//Set the remaining wheels.
	/*for(physx::PxU32 i = 2, wheelCount = 4; i < numWheels-2; i+=2, wheelCount+=2)
	{
		wheelCentreOffsets[wheelCount + 0] = physx::PxVec3((-chassisDims.x + wheelWidth)*0.5f, -(chassisDims.y/2 + wheelRadius), wheelRearZ + i*deltaZ*0.5f) *scale;
		wheelCentreOffsets[wheelCount + 1] = physx::PxVec3((+chassisDims.x - wheelWidth)*0.5f, -(chassisDims.y/2 + wheelRadius), wheelRearZ + i*deltaZ*0.5f) *scale;
	}*/
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
		PxVehicleComputeSprungMasses
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

physx::PxVehicleDrive4W* createVehicle4W(const VehicleDesc& vehicleDesc, physx::PxPhysics* physics, physx::PxCooking* cooking)
{
	const physx::PxVec3 chassisDims = vehicleDesc.chassisDims;
	const physx::PxF32 wheelWidth = vehicleDesc.wheelWidth;
	const physx::PxF32 wheelRadius = vehicleDesc.wheelRadius;
	const physx::PxU32 numWheels = vehicleDesc.numWheels;

	const physx::PxFilterData& chassisSimFilterData = vehicleDesc.chassisSimFilterData;
	const physx::PxFilterData& wheelSimFilterData = vehicleDesc.wheelSimFilterData;

	//Construct a physx actor with shapes for the chassis and wheels.
	//Set the rigid body mass, moment of inertia, and center of mass offset.
	physx::PxRigidDynamic* veh4WActor = NULL;
	{
		//Construct a convex mesh for a cylindrical wheel.
		physx::PxConvexMesh* wheelMesh = createWheelMesh(wheelWidth, wheelRadius, *physics, *cooking);
		//Assume all wheels are identical for simplicity.
		physx::PxConvexMesh* wheelConvexMeshes[PX_MAX_NB_WHEELS];
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
		physx::PxConvexMesh* chassisConvexMesh = createChassisMesh(chassisDims, *physics, *cooking);
		physx::PxConvexMesh* chassisConvexMeshes[1] = {chassisConvexMesh};
		physx::PxMaterial* chassisMaterials[1] = {vehicleDesc.chassisMaterial};

		//Rigid body data.
		physx::PxVehicleChassisData rigidBodyData;
		rigidBodyData.mMOI = vehicleDesc.chassisMOI;
		rigidBodyData.mMass = vehicleDesc.chassisMass;
		rigidBodyData.mCMOffset = vehicleDesc.chassisCMOffset;

		veh4WActor = createVehicleActor
		(rigidBodyData,
			wheelMaterials, wheelConvexMeshes, numWheels, wheelSimFilterData,
			chassisMaterials, chassisConvexMeshes, 1, chassisSimFilterData,
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
			vehicleDesc.chassisCMOffset *scale, vehicleDesc.chassisMass,
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
	vehDrive4W->setup(physics, veh4WActor, *wheelsSimData, driveSimData, numWheels - 4);
	//Configure the userdata
	configureUserData(vehDrive4W, vehicleDesc.actorUserData, vehicleDesc.shapeUserDatas);

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
	m_surfaceTirePairs = px_create_unique_ptr(physx::PxVehicleDrivableSurfaceToTireFrictionPairs::allocate(numTireTypes,numSurfaceTypes));
	assert(materials.size() == numSurfaceTypes);
	m_surfaceTirePairs->setup(numTireTypes,numSurfaceTypes,const_cast<const physx::PxMaterial**>(static_cast<physx::PxMaterial**>(materials.data())),drivableSurfaceTypes.data());//,);
	/*for(physx::PxU32 i=0;i<numSurfaceTypes;i++)
	{
	for(physx::PxU32 j=0;j<numTireTypes;j++)
	{
	m_surfaceTirePairs->setTypePairFriction(i,j,TireFrictionMultipliers::getValue(i, j));
	}
	//																																									  }*/

	// Input
	/*std::vector<pragma::physics::WheelCreateInfo> wheels;
	std::vector<pragma::physics::TireCreateInfo> tires;
	//

	struct PhysXWheelData
	{
		physx::PxVehicleWheelData wheel;
		physx::PxVehicleTireData tire;
		physx::PxVehicleSuspensionData suspension;

		physx::PxVec3 suspensionTravelDirection;
		physx::PxVec3 wheelCenterCMOffset;
		physx::PxVec3 suspensionForceAppCMOffset;
		physx::PxVec3 tireForceAppCMOffset;
	};
	auto numWheels = wheels.size();
	std::vector<PhysXWheelData> pxWheels {};
	pxWheels.reserve(numWheels);
	for(auto i=decltype(numWheels){0u};i<numWheels;++i)
	{
		pxWheels.push_back({});
		auto &wheelData = pxWheels.back();
		wheelData.wheel = ToPxWheelData(wheels.at(i));
		wheelData.tire = ToPxTireData(tires.at(i));
	}

	std::vector<physx::PxF32> sprungMasses {};
	sprungMasses.resize(numWheels);
	physx::PxVehicleComputeSprungMasses(numWheels,wheelCenterActorOffsets,chassisCMOffset,chassisMass,1,suspSprungMasses);

	constexpr physx::PxF32 camberAngleAtRest = 0.0;
	constexpr physx::PxF32 camberAngleAtMaxDroop = 0.01f;
	constexpr physx::PxF32 camberAngleAtMaxCompression = -0.01f;
	auto *wheelsSimData = physx::PxVehicleWheelsSimData::allocate(numWheels);

	physx::PxFilterData queryFilterData;
	queryFilterData.word3 = UNDRIVABLE_SURFACE;
	for(auto i=decltype(numWheels){0u};i<numWheels;++i)
	{
		auto &wheelData = pxWheels.at(i);
		auto &suspension = wheelData.suspension;
		suspension.mMaxCompression = 0.3f;
		suspension.mMaxDroop = 0.1f;
		suspension.mSpringStrength = 35'000.0f;
		suspension.mSpringDamperRate = 4'500.0f;
		suspension.mSprungMass = suspSprungMasses.at(i);

		auto factor = ((i %2) == 0) ? 1.f : -1.f;
		suspension.mCamberAtRest = camberAngleAtRest *factor;
		suspension.mCamberAtMaxDroop = camberAngleAtMaxDroop *factor;
		suspension.mCamberAtMaxCompression = camberAngleAtMaxCompression *factor;

		//Vertical suspension travel.
		wheelData.suspensionTravelDirection = physx::PxVec3(0,-1,0);

		//Wheel center offset is offset from rigid body center of mass.
		wheelData.wheelCenterCMOffset = wheelCenterActorOffsets -chassisCMOffset;

		//Suspension force application point 0.3 metres below
		//rigid body center of mass.
		wheelData.suspensionForceAppCMOffset = physx::PxVec3(wheelData.wheelCenterCMOffset.x,-0.3f,wheelData.wheelCenterCMOffset.z);

		//Tire force application point 0.3 metres below
		//rigid body center of mass.
		wheelData.tireForceAppCMOffset = physx::PxVec3(wheelData.wheelCenterCMOffset.x,-0.3f,wheelData.wheelCenterCMOffset.z);

		wheelsSimData->setWheelData(i,wheelData.wheel);
		wheelsSimData->setTireData(i,wheelData.tire);
		wheelsSimData->setSuspensionData(i,wheelData.suspension);
		wheelsSimData->setSuspTravelDirection(i,wheelData.suspensionTravelDirection);
		wheelsSimData->setWheelCentreOffset(i,wheelData.wheelCenterCMOffset);
		wheelsSimData->setSuspForceAppPointOffset(i,wheelData.suspensionForceAppCMOffset);
		wheelsSimData->setTireForceAppPointOffset(i,wheelData.tireForceAppCMOffset);
		wheelsSimData->setSceneQueryFilterData(i,queryFilterData);
		wheelsSimData->setWheelShapeMapping(i,i);
	}

	physx::PxVehicleDriveSimData4W driveSimData {};
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
			wheelsSimData->getWheelCentreOffset(physx::PxVehicleDrive4WWheelOrder::eREAR_RIGHT).x-
			wheelsSimData->getWheelCentreOffset(physx::PxVehicleDrive4WWheelOrder::eREAR_LEFT).x;
		driveSimData.setAckermannGeometryData(ackermann);

		physx::PxTransform startPose {physx::PxIdentity};
		physx::PxRigidDynamic* vehActor = GetPhysics().createRigidDynamic(startPose);
		setupVehicleActor(vehActor);
		m_scene->addActor(*vehActor);

		physx::PxVehicleDrive4W* vehDrive4W = physx::PxVehicleDrive4W::allocate(numWheels);
		vehDrive4W->setup(&GetPhysics(), vehActor, *wheelsSimData, driveSimData, numWheels - 4);
		wheelsSimData->free();
	}

	*/
	auto *gPhysics = &GetPhysics();
	auto *gCooking = &GetCooking();
	auto *gScene = &GetScene();
	auto *physics = gPhysics;
	auto *cooking = gCooking;

	gMaterial = std::dynamic_pointer_cast<PhysXMaterial>(CreateMaterial(0.5f,0.5f,0.6f));

	VehicleDesc vehicleDesc = initVehicleDesc();
	auto gVehicle = px_create_unique_ptr<physx::PxVehicleDrive>(createVehicle4W(vehicleDesc, gPhysics, gCooking));
	auto *gVehicle4W = static_cast<physx::PxVehicleDrive4W*>(gVehicle.get());
	physx::PxTransform startTransform(physx::PxVec3(0, (vehicleDesc.chassisDims.y*0.5f + vehicleDesc.wheelRadius + 1.0f), 0), physx::PxQuat(physx::PxIdentity));


/*
std::shared_ptr<PhysXConvexShape> shape = nullptr;
switch(internalShape.getGeometryType())
{
case physx::PxGeometryType::eBOX:
{
// Geometry and shape will automatically be removed by controller
auto geometry = std::unique_ptr<physx::PxGeometry,void(*)(physx::PxGeometry*)>{&internalShape.getGeometry().box(),[](physx::PxGeometry*) {}};
auto pxShape = std::unique_ptr<physx::PxShape,void(*)(physx::PxShape*)>{&internalShape,[](physx::PxShape*) {}};
shape = CreateSharedPtr<PhysXConvexShape>(*this,std::move(pxShape),std::move(geometry));
InitializeShape(*shape,true);
break;
}
case physx::PxGeometryType::eCAPSULE:
{
// Geometry and shape will automatically be removed by controller
auto geometry = std::unique_ptr<physx::PxGeometry,void(*)(physx::PxGeometry*)>{&internalShape.getGeometry().capsule(),[](physx::PxGeometry*) {}};
auto pxShape = std::unique_ptr<physx::PxShape,void(*)(physx::PxShape*)>{&internalShape,[](physx::PxShape*) {}};
shape = CreateSharedPtr<PhysXConvexShape>(*this,std::move(pxShape),std::move(geometry));
InitializeShape(*shape,true);
break;
}
default:
return nullptr;
}
if(shape == nullptr)
return nullptr;


*/
	// Actor will automatically be removed by controller
	auto rigidDynamic = std::unique_ptr<physx::PxActor,void(*)(physx::PxActor*)>{gVehicle4W->getRigidDynamicActor(),[](physx::PxActor*) {}};
	auto *pRigidDynamic = static_cast<physx::PxRigidDynamic*>(rigidDynamic.get());

	physx::PxShape *shape;
	pRigidDynamic->getShapes(&shape,1);

	// Geometry and shape will automatically be removed by controller
	auto geometry = std::unique_ptr<physx::PxGeometry,void(*)(physx::PxGeometry*)>{&shape->getGeometry().convexMesh(),[](physx::PxGeometry*) {}};
	auto pxShape = std::unique_ptr<physx::PxShape,void(*)(physx::PxShape*)>{shape,[](physx::PxShape*) {}};
	auto convexShape = CreateSharedPtr<PhysXConvexShape>(*this,std::move(pxShape),std::move(geometry));
	InitializeShape(*convexShape,true);

	auto rigidBody = CreateSharedHandle<PhysXRigidDynamic>(*this,std::move(rigidDynamic),*convexShape,pRigidDynamic->getMass(),FromPhysXVector(pRigidDynamic->getMassSpaceInertiaTensor()));
	InitializeCollisionObject(*rigidBody);

	gVehicle4W->getRigidDynamicActor()->setGlobalPose(startTransform);
	gScene->addActor(*gVehicle4W->getRigidDynamicActor());

	//Set the vehicle to rest in first gear.
	//Set the vehicle to use auto-gears.
	gVehicle4W->setToRestState();
	gVehicle4W->mDriveDynData.forceGearChange(physx::PxVehicleGearsData::eFIRST);
	gVehicle4W->mDriveDynData.setUseAutoGears(true);

	auto vhc = CreateSharedHandle<PhysXVehicle>(*this,std::move(gVehicle),util::shared_handle_cast<PhysXRigidDynamic,ICollisionObject>(rigidBody));
	AddVehicle(*vhc);
	return util::shared_handle_cast<PhysXVehicle,IVehicle>(vhc);

#if 0

	const physx::PxVec3 chassisDims = vehicleDesc.chassisDims;
	const physx::PxF32 wheelWidth = vehicleDesc.wheelWidth;
	const physx::PxF32 wheelRadius = vehicleDesc.wheelRadius;
	const physx::PxU32 numWheels = vehicleDesc.numWheels;

	const physx::PxFilterData& chassisSimFilterData = vehicleDesc.chassisSimFilterData;
	const physx::PxFilterData& wheelSimFilterData = vehicleDesc.wheelSimFilterData;

	//Construct a physx actor with shapes for the chassis and wheels.
	//Set the rigid body mass, moment of inertia, and center of mass offset.
	physx::PxRigidDynamic* veh4WActor = NULL;
	{
		//Construct a convex mesh for a cylindrical wheel.
		physx::PxConvexMesh* wheelMesh = createWheelMesh(wheelWidth, wheelRadius, *physics, *cooking);
		//Assume all wheels are identical for simplicity.
		physx::PxConvexMesh* wheelConvexMeshes[PX_MAX_NB_WHEELS];
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
		physx::PxConvexMesh* chassisConvexMesh = createChassisMesh(chassisDims, *physics, *cooking);
		physx::PxConvexMesh* chassisConvexMeshes[1] = {chassisConvexMesh};
		physx::PxMaterial* chassisMaterials[1] = {vehicleDesc.chassisMaterial};

		//Rigid body data.
		physx::PxVehicleChassisData rigidBodyData;
		rigidBodyData.mMOI = vehicleDesc.chassisMOI;
		rigidBodyData.mMass = vehicleDesc.chassisMass;
		rigidBodyData.mCMOffset = vehicleDesc.chassisCMOffset;

		veh4WActor = createVehicleActor
		(rigidBodyData,
			wheelMaterials, wheelConvexMeshes, numWheels, wheelSimFilterData,
			chassisMaterials, chassisConvexMeshes, 1, chassisSimFilterData,
			*physics);
	}

	//Set up the sim data for the wheels.
	physx::PxVehicleWheelsSimData* wheelsSimData = physx::PxVehicleWheelsSimData::allocate(numWheels);
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
			wheelsSimData);
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
	auto vehDrive = px_create_unique_ptr<physx::PxVehicleDrive>(physx::PxVehicleDrive4W::allocate(numWheels));
	auto *vehDrive4W = static_cast<physx::PxVehicleDrive4W*>(vehDrive.get());
	vehDrive4W->setup(physics, veh4WActor, *wheelsSimData, driveSimData, numWheels - 4);

	//Configure the userdata
	configureUserData(vehDrive4W, vehicleDesc.actorUserData, vehicleDesc.shapeUserDatas);

	//Free the sim data because we don't need that any more.
	wheelsSimData->free();

	//return vehDrive4W;
#endif


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

	//IEnvironment &env,PhysXUniquePtr<physx::PxVehicleDrive> vhc);
}
