#include "pr_module.hpp"
#include "pr_physx/environment.hpp"
#include "pr_physx/shape.hpp"
#include "pr_physx/collision_object.hpp"
#include "pr_physx/constraint.hpp"
#include "pr_physx/controller.hpp"
#include "pr_physx/material.hpp"
#include "pr_physx/debug.hpp"
#include "pr_physx/raycast.hpp"
#include <sharedutils/util.h>
#include <pragma/physics/transform.hpp>
#include <pragma/physics/raytraces.h>
#include <pragma/entities/baseentity.h>
#include <PxPhysicsAPI.h>
#include <PxFoundation.h>
#include <vehicle/PxVehicleSDK.h>
#ifdef _WIN32
#include <common/windows/PxWindowsDelayLoadHook.h>
#endif

#pragma optimize("",off)
pragma::physics::Transform pragma::physics::PxEnvironment::CreateTransform(const physx::PxTransform &pxTransform)
{
	return Transform {uvec::create(pxTransform.p),uquat::create(pxTransform.q)};
}
physx::PxTransform pragma::physics::PxEnvironment::CreatePxTransform(const Transform &t)
{
	return physx::PxTransform {uvec::create_px(t.GetOrigin()),uquat::create_px(t.GetRotation())};
}
pragma::physics::PxCollisionObject *pragma::physics::PxEnvironment::GetCollisionObject(const physx::PxRigidActor &actor)
{
	return static_cast<pragma::physics::PxCollisionObject*>(actor.userData);
}
pragma::physics::PxShape *pragma::physics::PxEnvironment::GetShape(const physx::PxShape &shape)
{
	return static_cast<pragma::physics::PxShape*>(shape.userData);
}
pragma::physics::PxEnvironment::PxEnvironment(NetworkState &state)
	: IEnvironment{state}
{}
pragma::physics::PxEnvironment::~PxEnvironment()
{
	m_cooking = nullptr;
	m_scene = nullptr;
	m_physics = nullptr;
	m_pvd = nullptr;
}

static physx::PxDefaultErrorCallback gDefaultErrorCallback {};
static physx::PxDefaultAllocator gDefaultAllocatorCallback {};

extern "C"
{
	PRAGMA_EXPORT void initialize_physics_engine(NetworkState &nw,std::unique_ptr<pragma::physics::IEnvironment> &outEnv);
};

#ifdef _WIN32
class PxDelayLoadHook
	: public physx::PxDelayLoadHook
{
	std::string get_library_path(const std::string &libName) const
	{
		auto path = util::get_path_to_library(initialize_physics_engine);
		if(path.has_value() == false)
			return libName;
		return *path +'/' +libName;
	}
	virtual const char* getPhysXCommonDllName() const
	{
		return get_library_path("PhysXCommon_64.dll").c_str();
	}

	virtual const char* getPhysXFoundationDllName() const
	{
		return get_library_path("PhysXFoundation_64.dll").c_str();
	}
} static g_DelayLoadHook;
#endif

static uint32_t g_instances = 0;
static pragma::physics::PxUniquePtr<physx::PxFoundation> g_pxFoundation = pragma::physics::px_null_ptr<physx::PxFoundation>();
extern "C"
{
	PRAGMA_EXPORT bool pragma_attach(std::string&)
	{
		if(g_instances++ > 0)
			return true;
#ifdef _WIN32
		PxSetPhysXDelayLoadHook(&g_DelayLoadHook);
		PxSetPhysXCookingDelayLoadHook(&g_DelayLoadHook);
		PxSetPhysXCommonDelayLoadHook(&g_DelayLoadHook);
#endif
		g_pxFoundation = {PxCreateFoundation(PX_PHYSICS_VERSION,gDefaultAllocatorCallback,gDefaultErrorCallback),[](physx::PxFoundation *pFoundation) {
			if(pFoundation)
				pFoundation->release();
		}};
		return true;
	}
	PRAGMA_EXPORT void pragma_detach()
	{
		if(--g_instances > 0)
			return;
		PxCloseExtensions();
		physx::PxCloseVehicleSDK();
		g_pxFoundation = nullptr;
	}
};

bool pragma::physics::PxEnvironment::Initialize()
{
	if(g_pxFoundation == nullptr)
		return false;
	m_pvd = px_create_unique_ptr(physx::PxCreatePvd(*g_pxFoundation));
	if(m_pvd == nullptr)
		return false;
	auto *pTransport = physx::PxDefaultPvdSocketTransportCreate("127.0.0.1",5425,10);
	m_pvd->connect(*pTransport,physx::PxPvdInstrumentationFlag::eALL);

	physx::PxTolerancesScale scale;
	scale.length = 100;        // typical length of an object
	scale.speed = 981;         // typical speed of an object, gravity*1s is a reasonable choice
	m_physics = px_create_unique_ptr(PxCreatePhysics(PX_PHYSICS_VERSION,*g_pxFoundation,scale));
	if(m_physics == nullptr)
		return false;

	physx::PxSceneDesc sceneDesc {scale};
	// TODO: Scene settings!
	m_scene = px_create_unique_ptr(m_physics->createScene(sceneDesc));
	if(m_scene == nullptr)
		return false;
	m_scene->setGravity({0.f,0.f,0.f});
	m_cooking = px_create_unique_ptr(PxCreateCooking(PX_PHYSICS_VERSION,*g_pxFoundation,physx::PxCookingParams(scale)));
	if(m_cooking == nullptr)
		return false;

	m_controllerManager = px_create_unique_ptr(PxCreateControllerManager(*m_scene));
	if(m_controllerManager == nullptr)
		return false;
	m_controllerManager->setOverlapRecoveryModule(true);
	if(PxInitExtensions(*m_physics,m_pvd.get()) == false || physx::PxInitVehicleSDK(*m_physics) == false)
		return false;
	physx::PxVehicleSetBasisVectors(ToPhysXNormal(uvec::UP),ToPhysXNormal(uvec::FORWARD));
	physx::PxVehicleSetUpdateMode(physx::PxVehicleUpdateMode::eVELOCITY_CHANGE);
	return true;
}
physx::PxVec3 pragma::physics::PxEnvironment::ToPhysXVector(const Vector3 &v) const
{
	return physx::PxVec3{v.x,v.y,v.z};
}
physx::PxVec3 pragma::physics::PxEnvironment::ToPhysXNormal(const Vector3 &n) const
{
	return physx::PxVec3{n.x,n.y,n.z};
}
physx::PxVec3 pragma::physics::PxEnvironment::ToPhysXTorque(const Vector3 &t) const
{
	return physx::PxVec3{t.x,t.y,t.z};
}
physx::PxQuat pragma::physics::PxEnvironment::ToPhysXRotation(const Quat &rot) const
{
	return physx::PxQuat{rot.x,rot.y,rot.z,rot.w};
}
Vector3 pragma::physics::PxEnvironment::FromPhysXVector(const physx::PxVec3 &v) const
{
	return Vector3{v.x,v.y,v.z};
}
Vector3 pragma::physics::PxEnvironment::FromPhysXNormal(const physx::PxVec3 &n) const
{
	return Vector3{n.x,n.y,n.z};
}
Vector3 pragma::physics::PxEnvironment::FromPhysXTorque(const physx::PxVec3 &t) const
{
	return Vector3{t.x,t.y,t.z};
}
Quat pragma::physics::PxEnvironment::FromPhysXRotation(const physx::PxQuat &v) const
{
	return Quat{v.w,v.x,v.y,v.z};
}
pragma::physics::PxRigidBody &pragma::physics::PxEnvironment::ToBtType(IRigidBody &body) {return dynamic_cast<PxRigidBody&>(body);}
pragma::physics::IVisualDebugger *pragma::physics::PxEnvironment::InitializeVisualDebugger()
{
	auto visDebugger = std::make_shared<PxVisualDebugger>();
	m_visualDebugger = visDebugger;
	return m_visualDebugger.get();
}
physx::PxScene &pragma::physics::PxEnvironment::GetScene() const {return *m_scene;}
double pragma::physics::PxEnvironment::ToPhysXLength(double len) const {return len;}
double pragma::physics::PxEnvironment::FromPhysXLength(double len) const {return len;}
util::TSharedHandle<pragma::physics::IFixedConstraint> pragma::physics::PxEnvironment::CreateFixedConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB)
{
	physx::PxTransform tA {uvec::create_px(pivotA),uquat::create_px(rotA)};
	physx::PxTransform tB {uvec::create_px(pivotB),uquat::create_px(rotB)};
	auto fixedJoint = px_create_unique_ptr(physx::PxFixedJointCreate(*m_physics,&ToBtType(a).GetInternalObject(),tA,&ToBtType(b).GetInternalObject(),tB));
	return util::shared_handle_cast<PxFixedConstraint,IFixedConstraint>(CreateSharedHandle<PxFixedConstraint>(*this,std::move(fixedJoint)));
}
util::TSharedHandle<pragma::physics::IBallSocketConstraint> pragma::physics::PxEnvironment::CreateBallSocketConstraint(IRigidBody &a,const Vector3 &pivotA,IRigidBody &b,const Vector3 &pivotB)
{
	physx::PxTransform tA {uvec::create_px(pivotA)};
	physx::PxTransform tB {uvec::create_px(pivotB)};
	auto sphericalJoint = px_create_unique_ptr(physx::PxSphericalJointCreate(*m_physics,&ToBtType(a).GetInternalObject(),tA,&ToBtType(b).GetInternalObject(),tB));
	return util::shared_handle_cast<PxBallSocketConstraint,IBallSocketConstraint>(CreateSharedHandle<PxBallSocketConstraint>(*this,std::move(sphericalJoint)));
}
util::TSharedHandle<pragma::physics::IHingeConstraint> pragma::physics::PxEnvironment::CreateHingeConstraint(IRigidBody &a,const Vector3 &pivotA,IRigidBody &b,const Vector3 &pivotB,const Vector3 &axis)
{
	physx::PxTransform tA {uvec::create_px(pivotA)};
	physx::PxTransform tB {uvec::create_px(pivotB)};
	auto hingeJoint = px_create_unique_ptr(physx::PxRevoluteJointCreate(*m_physics,&ToBtType(a).GetInternalObject(),tA,&ToBtType(b).GetInternalObject(),tB));
	// TODO: Specify axis?
	return util::shared_handle_cast<PxHingeConstraint,IHingeConstraint>(CreateSharedHandle<PxHingeConstraint>(*this,std::move(hingeJoint)));
}
util::TSharedHandle<pragma::physics::ISliderConstraint> pragma::physics::PxEnvironment::CreateSliderConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB)
{
	physx::PxTransform tA {uvec::create_px(pivotA),uquat::create_px(rotA)};
	physx::PxTransform tB {uvec::create_px(pivotB),uquat::create_px(rotB)};
	auto prismaticJoint = px_create_unique_ptr(physx::PxPrismaticJointCreate(*m_physics,&ToBtType(a).GetInternalObject(),tA,&ToBtType(b).GetInternalObject(),tB));
	return util::shared_handle_cast<PxSliderConstraint,ISliderConstraint>(CreateSharedHandle<PxSliderConstraint>(*this,std::move(prismaticJoint)));
}
util::TSharedHandle<pragma::physics::IConeTwistConstraint> pragma::physics::PxEnvironment::CreateConeTwistConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB)
{
	physx::PxTransform tA {uvec::create_px(pivotA),uquat::create_px(rotA)};
	physx::PxTransform tB {uvec::create_px(pivotB),uquat::create_px(rotB)};
	auto sphericalJoint = px_create_unique_ptr(physx::PxSphericalJointCreate(*m_physics,&ToBtType(a).GetInternalObject(),tA,&ToBtType(b).GetInternalObject(),tB));
	return util::shared_handle_cast<PxConeTwistConstraint,IConeTwistConstraint>(CreateSharedHandle<PxConeTwistConstraint>(*this,std::move(sphericalJoint)));
}
util::TSharedHandle<pragma::physics::IDoFConstraint> pragma::physics::PxEnvironment::CreateDoFConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB)
{
	physx::PxTransform tA {uvec::create_px(pivotA),uquat::create_px(rotA)};
	physx::PxTransform tB {uvec::create_px(pivotB),uquat::create_px(rotB)};
	auto d6Joint = px_create_unique_ptr(physx::PxD6JointCreate(*m_physics,&ToBtType(a).GetInternalObject(),tA,&ToBtType(b).GetInternalObject(),tB));
	return util::shared_handle_cast<PxDoFConstraint,IDoFConstraint>(CreateSharedHandle<PxDoFConstraint>(*this,std::move(d6Joint)));
}
util::TSharedHandle<pragma::physics::IDoFSpringConstraint> pragma::physics::PxEnvironment::CreateDoFSpringConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB)
{
	// TODO?
	return nullptr;
}
void pragma::physics::PxEnvironment::InitializeControllerDesc(physx::PxControllerDesc &inOutDesc,float stepHeight,float slopeLimitDeg,const Transform &startTransform)
{
	auto pos = ToPhysXVector(startTransform.GetOrigin());
	inOutDesc.contactOffset = 0.1f; // TODO: Scale
	inOutDesc.density = 10.f; // TODO
	inOutDesc.invisibleWallHeight = 0.f;
	inOutDesc.material = &dynamic_cast<PxMaterial&>(GetGenericMaterial()).GetPxMaterial();
	inOutDesc.maxJumpHeight = 0.0;
	inOutDesc.nonWalkableMode = physx::PxControllerNonWalkableMode::ePREVENT_CLIMBING_AND_FORCE_SLIDING;
	inOutDesc.scaleCoeff = 0.8f;
	inOutDesc.slopeLimit = umath::cos(umath::deg_to_rad(slopeLimitDeg));
	inOutDesc.stepOffset = ToPhysXLength(stepHeight);
	inOutDesc.position = physx::PxExtendedVec3{pos.x,pos.y,pos.z};
	inOutDesc.upDirection = ToPhysXVector(uquat::up(startTransform.GetRotation()));
	inOutDesc.volumeGrowth = 1.5f;
}
util::TSharedHandle<pragma::physics::IController> pragma::physics::PxEnvironment::CreateController(PxUniquePtr<physx::PxController> c)
{
	auto *pActor = c->getActor();
	physx::PxShape *shapes;
	if(pActor == nullptr || pActor->getShapes(&shapes,1) == 0)
		return nullptr;
	auto &internalShape = shapes[0];
	std::shared_ptr<PxConvexShape> shape = nullptr;
	switch(internalShape.getGeometryType())
	{
	case physx::PxGeometryType::eBOX:
	{
		// Geometry and shape will automatically be removed by controller
		auto geometry = std::unique_ptr<physx::PxGeometry,void(*)(physx::PxGeometry*)>{&internalShape.getGeometry().box(),[](physx::PxGeometry*) {}};
		auto pxShape = std::unique_ptr<physx::PxShape,void(*)(physx::PxShape*)>{&internalShape,[](physx::PxShape*) {}};
		shape = CreateSharedPtr<PxConvexShape>(*this,std::move(pxShape),std::move(geometry));
		InitializeShape(*shape,true);
		break;
	}
	case physx::PxGeometryType::eCAPSULE:
	{
		// Geometry and shape will automatically be removed by controller
		auto geometry = std::unique_ptr<physx::PxGeometry,void(*)(physx::PxGeometry*)>{&internalShape.getGeometry().capsule(),[](physx::PxGeometry*) {}};
		auto pxShape = std::unique_ptr<physx::PxShape,void(*)(physx::PxShape*)>{&internalShape,[](physx::PxShape*) {}};
		shape = CreateSharedPtr<PxConvexShape>(*this,std::move(pxShape),std::move(geometry));
		InitializeShape(*shape,true);
		break;
	}
	default:
		return nullptr;
	}
	if(shape == nullptr)
		return nullptr;

	// Actor will automatically be removed by controller
	auto pRigidDynamic = std::unique_ptr<physx::PxRigidDynamic,void(*)(physx::PxRigidDynamic*)>{pActor,[](physx::PxRigidDynamic*) {}};
	auto rigidBody = CreateSharedHandle<PxRigidBody>(*this,std::move(pRigidDynamic),*shape,pRigidDynamic->getMass(),FromPhysXVector(pRigidDynamic->getMassSpaceInertiaTensor()));
	pRigidDynamic->userData = static_cast<pragma::physics::PxCollisionObject*>(rigidBody.Get());
	InitializeCollisionObject(*rigidBody);

	return util::shared_handle_cast<PxController,IController>(
		CreateSharedHandle<pragma::physics::PxController>(*this,std::move(c),util::shared_handle_cast<PxRigidBody,ICollisionObject>(rigidBody))
	);
}
util::TSharedHandle<pragma::physics::IController> pragma::physics::PxEnvironment::CreateCapsuleController(float halfWidth,float halfHeight,float stepHeight,float slopeLimitDeg,const Transform &startTransform)
{
	physx::PxCapsuleControllerDesc capsuleDesc {};
	InitializeControllerDesc(capsuleDesc,stepHeight,slopeLimitDeg,startTransform);
	capsuleDesc.climbingMode = physx::PxCapsuleClimbingMode::eEASY;
	capsuleDesc.height = halfHeight *2.f;
	capsuleDesc.radius = halfWidth;
	
	auto c = px_create_unique_ptr(m_controllerManager->createController(capsuleDesc));
	return c ? CreateController(std::move(c)) : nullptr;
}
util::TSharedHandle<pragma::physics::IController> pragma::physics::PxEnvironment::CreateBoxController(const Vector3 &halfExtents,float stepHeight,float slopeLimitDeg,const Transform &startTransform)
{
	physx::PxBoxControllerDesc boxDesc {};
	InitializeControllerDesc(boxDesc,stepHeight,slopeLimitDeg,startTransform);
	boxDesc.halfHeight = halfExtents.y;
	boxDesc.halfSideExtent = halfExtents.z;
	boxDesc.halfForwardExtent = halfExtents.x;

	auto c = px_create_unique_ptr(m_controllerManager->createController(boxDesc));
	return c ? CreateController(std::move(c)) : nullptr;
}
util::TSharedHandle<pragma::physics::ICollisionObject> pragma::physics::PxEnvironment::CreateCollisionObject(IShape &shape)
{
	return nullptr;
}
util::TSharedHandle<pragma::physics::IRigidBody> pragma::physics::PxEnvironment::CreateRigidBody(float mass,IShape &shape,const Vector3 &localInertia)
{
	physx::PxTransform t {physx::PxVec3{0.f,0.f,0.f},physx::PxQuat{1.f}};
	auto pRigidDynamic = px_create_unique_ptr(m_physics->createRigidDynamic(t));
	if(pRigidDynamic == nullptr)
		return nullptr;
	auto rigidBody = CreateSharedHandle<PxRigidBody>(*this,std::move(pRigidDynamic),shape,mass,localInertia);
	if(pRigidDynamic->attachShape(const_cast<PxShape&>(dynamic_cast<const PxShape&>(shape)).GetInternalObject()) == false)
		return nullptr;
	pRigidDynamic->userData = static_cast<pragma::physics::PxCollisionObject*>(rigidBody.Get());
	auto density = 1.f; // TODO
	physx::PxRigidBodyExt::updateMassAndInertia(*pRigidDynamic,density);
	m_scene->addActor(*pRigidDynamic);
	InitializeCollisionObject(*rigidBody);
	return util::shared_handle_cast<PxRigidBody,IRigidBody>(rigidBody);
}
util::TSharedHandle<pragma::physics::ISoftBody> pragma::physics::PxEnvironment::CreateSoftBody(const PhysSoftBodyInfo &info,float mass,const std::vector<Vector3> &verts,const std::vector<uint16_t> &indices,std::vector<uint16_t> &indexTranslations)
{
	return nullptr;
}
util::TSharedHandle<pragma::physics::IGhostObject> pragma::physics::PxEnvironment::CreateGhostObject(IShape &shape)
{
	return nullptr;
}

void pragma::physics::PxEnvironment::InitializeShape(PxShape &shape,bool basicOnly)
{
	shape.GetInternalObject().userData = &shape;
	if(basicOnly)
		return;
	shape.GetInternalObject().setFlag(physx::PxShapeFlag::eSIMULATION_SHAPE,true);
	shape.GetInternalObject().setFlag(physx::PxShapeFlag::eVISUALIZATION,true);

	physx::PxTransform t {physx::PxVec3{0.f,0.f,0.f},physx::PxQuat{1.f}};
	shape.GetInternalObject().setLocalPose(t);
}
void pragma::physics::PxEnvironment::InitializeCollisionObject(PxCollisionObject &o)
{
	o.GetInternalObject().setActorFlag(physx::PxActorFlag::eVISUALIZATION,true);
}
std::shared_ptr<pragma::physics::IConvexShape> pragma::physics::PxEnvironment::CreateCapsuleShape(float halfWidth,float halfHeight,const IMaterial &mat)
{
	auto geometry = std::unique_ptr<physx::PxGeometry,void(*)(physx::PxGeometry*)>(
		new physx::PxCapsuleGeometry{static_cast<physx::PxReal>(ToPhysXLength(halfWidth)),static_cast<physx::PxReal>(ToPhysXLength(halfHeight))},
		[](physx::PxGeometry *p) {delete p;}
	);
	auto pxShape = px_create_unique_ptr(m_physics->createShape(*geometry,dynamic_cast<const PxMaterial&>(mat).GetPxMaterial(),false));
	if(pxShape == nullptr)
		return nullptr;
	auto shape = CreateSharedPtr<PxConvexShape>(*this,std::move(pxShape),std::move(geometry));
	InitializeShape(*shape);
	return shape;
}
std::shared_ptr<pragma::physics::IConvexShape> pragma::physics::PxEnvironment::CreateBoxShape(const Vector3 &halfExtents,const IMaterial &mat)
{
	auto geometry = std::unique_ptr<physx::PxGeometry,void(*)(physx::PxGeometry*)>(
		new physx::PxBoxGeometry{ToPhysXVector(halfExtents)},
		[](physx::PxGeometry *p) {delete p;}
	);
	auto pxShape = px_create_unique_ptr(m_physics->createShape(*geometry,dynamic_cast<const PxMaterial&>(mat).GetPxMaterial(),false));
	if(pxShape == nullptr)
		return nullptr;
	auto shape = CreateSharedPtr<PxConvexShape>(*this,std::move(pxShape),std::move(geometry));
	InitializeShape(*shape);
	return shape;
}
std::shared_ptr<pragma::physics::IConvexShape> pragma::physics::PxEnvironment::CreateCylinderShape(float radius,float height,const IMaterial &mat)
{
	return nullptr;
}
std::shared_ptr<pragma::physics::ICompoundShape> pragma::physics::PxEnvironment::CreateTorusShape(uint32_t subdivisions,double outerRadius,double innerRadius,const IMaterial &mat)
{
	return nullptr;
}
std::shared_ptr<pragma::physics::IConvexShape> pragma::physics::PxEnvironment::CreateSphereShape(float radius,const IMaterial &mat)
{
	auto geometry = std::unique_ptr<physx::PxGeometry,void(*)(physx::PxGeometry*)>(
		new physx::PxSphereGeometry{static_cast<float>(ToPhysXLength(radius))},
		[](physx::PxGeometry *p) {delete p;}
	);
	auto pxShape = px_create_unique_ptr(m_physics->createShape(*geometry,dynamic_cast<const PxMaterial&>(mat).GetPxMaterial(),false));
	if(pxShape == nullptr)
		return nullptr;
	auto shape = CreateSharedPtr<PxConvexShape>(*this,std::move(pxShape),std::move(geometry));
	InitializeShape(*shape);
	return shape;
}
std::shared_ptr<pragma::physics::IConvexHullShape> pragma::physics::PxEnvironment::CreateConvexHullShape(const IMaterial &mat)
{
	return nullptr;
}
std::shared_ptr<pragma::physics::ITriangleShape> pragma::physics::PxEnvironment::CreateTriangleShape(const IMaterial &mat)
{
	return nullptr;
}
std::shared_ptr<pragma::physics::ICompoundShape> pragma::physics::PxEnvironment::CreateCompoundShape()
{
	return nullptr;
}
std::shared_ptr<pragma::physics::ICompoundShape> pragma::physics::PxEnvironment::CreateCompoundShape(IShape &shape)
{
	return nullptr;
}
std::shared_ptr<pragma::physics::ICompoundShape> pragma::physics::PxEnvironment::CreateCompoundShape(std::vector<IShape*> &shapes)
{
	return nullptr;
}
std::shared_ptr<pragma::physics::IShape> pragma::physics::PxEnvironment::CreateHeightfieldTerrainShape(uint32_t width,uint32_t length,Scalar maxHeight,uint32_t upAxis,const IMaterial &mat)
{
	return nullptr;
}
std::shared_ptr<pragma::physics::IMaterial> pragma::physics::PxEnvironment::CreateMaterial(float staticFriction,float dynamicFriction,float restitution)
{
	auto pMat = px_create_unique_ptr(m_physics->createMaterial(staticFriction,dynamicFriction,restitution));
	return CreateSharedPtr<PxMaterial>(*this,std::move(pMat));
}
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

pragma::physics::IEnvironment::RemainingDeltaTime pragma::physics::PxEnvironment::StepSimulation(float timeStep,int maxSubSteps,float fixedTimeStep)
{
	auto t = timeStep /fixedTimeStep;
	auto numSubSteps = umath::floor(t);
	for(auto i=decltype(numSubSteps){0u};i<numSubSteps;++i)
	{
		m_scene->simulate(fixedTimeStep);
		physx::PxU32 err;
		auto success = m_scene->fetchResults(true,&err);
		if(err)
			;
	}
	return fmodf(timeStep,fixedTimeStep);
}
void pragma::physics::PxEnvironment::InitializeRayCastResult(const TraceData &data,float rayLength,const physx::PxRaycastHit &raycastHit,TraceResult &outResult,RayCastHitType hitType) const
{
	auto *colObj = raycastHit.actor ? GetCollisionObject(*raycastHit.actor) : nullptr;
	if(colObj)
	{
		outResult.collisionObj = util::weak_shared_handle_cast<IBase,ICollisionObject>(colObj->GetHandle());
		auto *physObj = static_cast<PhysObj*>(colObj->userData);
		if(physObj)
		{
			outResult.physObj = physObj->GetHandle();
			auto *ent = outResult.physObj->GetOwner();
			outResult.entity = ent ? ent->GetEntity().GetHandle() : EntityHandle{};
		}
	}
	outResult.distance = FromPhysXLength(raycastHit.distance);
	outResult.fraction = rayLength /outResult.distance;
	outResult.hitType = hitType;
	outResult.normal = FromPhysXNormal(raycastHit.normal);
	outResult.position = FromPhysXVector(raycastHit.position);
	outResult.startPosition = data.GetSourceOrigin();
}

Bool pragma::physics::PxEnvironment::Overlap(const TraceData &data,std::vector<TraceResult> *optOutResults) const
{
	//m_scene->overlap();
	// TODO
	return false;
}

Bool pragma::physics::PxEnvironment::RayCast(const TraceData &data,std::vector<TraceResult> *optOutResults) const
{
	auto origin = ToPhysXVector(data.GetSourceOrigin());
	auto target = data.GetTargetOrigin();
	auto distance = uvec::length(target);
	if(distance == 0.f)
		return false;
	auto unitDir = ToPhysXVector(target);
	unitDir /= distance;
	physx::PxRaycastBuffer hit;
	
	auto flags = data.GetFlags();
	physx::PxQueryFlags queryFlags = physx::PxQueryFlag::eDYNAMIC | physx::PxQueryFlag::eSTATIC;
	auto hitFlags = static_cast<physx::PxHitFlags>(0);
	if(umath::is_flag_set(flags,RayCastFlags::ReportHitPosition))
		hitFlags |= physx::PxHitFlag::ePOSITION;
	if(umath::is_flag_set(flags,RayCastFlags::ReportHitNormal))
		hitFlags |= physx::PxHitFlag::eNORMAL;
	if(umath::is_flag_set(flags,RayCastFlags::ReportHitUV))
		hitFlags |= physx::PxHitFlag::eUV;
	if(umath::is_flag_set(flags,RayCastFlags::ReportAllResults))
		hitFlags |= physx::PxHitFlag::eMESH_MULTIPLE;
	if(umath::is_flag_set(flags,RayCastFlags::ReportAnyResult))
	{
		queryFlags = physx::PxQueryFlag::eANY_HIT;
		hitFlags |= physx::PxHitFlag::eMESH_ANY;
	}
	if(umath::is_flag_set(flags,RayCastFlags::ReportBackFaceHits))
		hitFlags |= physx::PxHitFlag::eMESH_BOTH_SIDES;
	if(umath::is_flag_set(flags,RayCastFlags::Precise))
		hitFlags |= physx::PxHitFlag::ePRECISE_SWEEP;

	if(umath::is_flag_set(flags,RayCastFlags::IgnoreDynamic))
		queryFlags &= ~physx::PxQueryFlag::eDYNAMIC;
	if(umath::is_flag_set(flags,RayCastFlags::IgnoreStatic))
		queryFlags &= ~physx::PxQueryFlag::eSTATIC;

	auto &filter = data.GetFilter();
	std::unique_ptr<RayCastFilterCallback> pxFilter = nullptr;
	if(filter)
	{
		pxFilter = std::make_unique<RayCastFilterCallback>(*this,*filter);
		if(filter->HasPreFilter())
			queryFlags |= physx::PxQueryFlag::ePREFILTER;
		if(filter->HasPostFilter())
			queryFlags |= physx::PxQueryFlag::ePOSTFILTER;
	}
	physx::PxQueryFilterData queryFilterData {queryFlags};
	auto bHitAny = m_scene->raycast(origin,unitDir,distance,hit,hitFlags,queryFilterData,pxFilter.get());
	if(optOutResults == nullptr || bHitAny == false)
		return bHitAny;
	auto numTouches = hit.getNbTouches();
	optOutResults->reserve(numTouches +1);
	for(auto i=decltype(numTouches){0u};i<numTouches;++i)
	{
		auto &touchHit = hit.getTouch(i);
		optOutResults->push_back({});
		auto &result = optOutResults->back();
		InitializeRayCastResult(data,distance,hit.block,result,RayCastHitType::Touch);
	}
	optOutResults->push_back({});
	auto &result = optOutResults->back();
	InitializeRayCastResult(data,distance,hit.block,result,bHitAny ? RayCastHitType::Block : RayCastHitType::None);
	return bHitAny;
}
Bool pragma::physics::PxEnvironment::Sweep(const TraceData &data,TraceResult *optOutResult) const
{
	auto *shape = data.GetShape();
	if(shape == nullptr)
		return false;
	physx::PxTransform pose {
		ToPhysXVector(data.GetSourceOrigin()),
		ToPhysXRotation(data.GetSourceRotation())
	};
	data.GetSourceRotation();
	auto target = data.GetTargetOrigin();
	auto distance = uvec::length(target);
	if(distance == 0.f)
		return false;
	auto unitDir = ToPhysXVector(target);
	unitDir /= distance;
	physx::PxRaycastBuffer hit;

	auto flags = data.GetFlags();
	physx::PxQueryFlags queryFlags = physx::PxQueryFlag::eDYNAMIC | physx::PxQueryFlag::eSTATIC;
	auto hitFlags = static_cast<physx::PxHitFlags>(0);
	if(umath::is_flag_set(flags,RayCastFlags::ReportHitPosition))
		hitFlags |= physx::PxHitFlag::ePOSITION;
	if(umath::is_flag_set(flags,RayCastFlags::ReportHitNormal))
		hitFlags |= physx::PxHitFlag::eNORMAL;
	if(umath::is_flag_set(flags,RayCastFlags::ReportHitUV))
		hitFlags |= physx::PxHitFlag::eUV;
	if(umath::is_flag_set(flags,RayCastFlags::ReportAllResults))
		hitFlags |= physx::PxHitFlag::eMESH_MULTIPLE;
	if(umath::is_flag_set(flags,RayCastFlags::ReportAnyResult))
	{
		queryFlags = physx::PxQueryFlag::eANY_HIT;
		hitFlags |= physx::PxHitFlag::eMESH_ANY;
	}
	if(umath::is_flag_set(flags,RayCastFlags::ReportBackFaceHits))
		hitFlags |= physx::PxHitFlag::eMESH_BOTH_SIDES;
	if(umath::is_flag_set(flags,RayCastFlags::Precise))
		hitFlags |= physx::PxHitFlag::ePRECISE_SWEEP;

	if(umath::is_flag_set(flags,RayCastFlags::IgnoreDynamic))
		queryFlags &= ~physx::PxQueryFlag::eDYNAMIC;
	if(umath::is_flag_set(flags,RayCastFlags::IgnoreStatic))
		queryFlags &= ~physx::PxQueryFlag::eSTATIC;

	physx::PxSweepCallback cb{}; // TODO

	auto &filter = data.GetFilter();
	std::unique_ptr<RayCastFilterCallback> pxFilter = nullptr;
	if(filter)
	{
		pxFilter = std::make_unique<RayCastFilterCallback>(*this,*filter);
		if(filter->HasPreFilter())
			queryFlags |= physx::PxQueryFlag::ePREFILTER;
		if(filter->HasPostFilter())
			queryFlags |= physx::PxQueryFlag::ePOSTFILTER;
	}
	void *userData = nullptr; -> Point to derived base type; avoid dynamic_cast
	GetDerivedType!
	static_cast<PxConvexShape*>(static_cast<PxShape*>(shape->userData)); TODO
	physx::PxQueryFilterData queryFilterData {queryFlags};
	auto bHitAny = m_scene->sweep(geometry,pose,unitDir,distance,hitCall,hitFlags,queryFilterData,pxFilter.get());
	if(optOutResults == nullptr || bHitAny == false)
		return bHitAny;
	auto numTouches = hit.getNbTouches();
	optOutResults->reserve(numTouches +1);
	for(auto i=decltype(numTouches){0u};i<numTouches;++i)
	{
		auto &touchHit = hit.getTouch(i);
		optOutResults->push_back({});
		auto &result = optOutResults->back();
		InitializeRayCastResult(data,distance,hit.block,result,RayCastHitType::Touch);
	}
	optOutResults->push_back({});
	auto &result = optOutResults->back();
	InitializeRayCastResult(data,distance,hit.block,result,bHitAny ? RayCastHitType::Block : RayCastHitType::None);
	return bHitAny;
}
#pragma optimize("",on)
