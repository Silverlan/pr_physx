#include "pr_module.hpp"
#include "pr_physx/environment.hpp"
#include "pr_physx/shape.hpp"
#include "pr_physx/collision_object.hpp"
#include "pr_physx/constraint.hpp"
#include "pr_physx/controller.hpp"
#include "pr_physx/material.hpp"
#include "pr_physx/raycast.hpp"
#include <sharedutils/util.h>
#include <pragma/math/surfacematerial.h>
#include <pragma/physics/transform.hpp>
#include <pragma/physics/visual_debugger.hpp>
#include <pragma/entities/baseentity.h>
#include <pragma/util/util_game.hpp>
#include <pragma/networkstate/networkstate.h>
#include <PxPhysicsAPI.h>
#include <PxFoundation.h>
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
pragma::physics::PxController *pragma::physics::PxEnvironment::GetController(const physx::PxController &controller)
{
	return static_cast<pragma::physics::PxController*>(controller.getUserData());
}
pragma::physics::PxMaterial *pragma::physics::PxEnvironment::GetMaterial(const physx::PxMaterial &material)
{
	return static_cast<pragma::physics::PxMaterial*>(material.userData);
}
pragma::physics::PxEnvironment::PxEnvironment(NetworkState &state)
	: IEnvironment{state}
{}
pragma::physics::PxEnvironment::~PxEnvironment()
{
	m_cooking = nullptr;
	m_scene = nullptr;
}

class PhysXErrorCallback
	: public physx::PxErrorCallback
{
public:
	virtual void reportError(physx::PxErrorCode::Enum code, const char* message, const char* file,int line) override
	{
		switch(code)
		{
		case physx::PxErrorCode::eDEBUG_INFO:
		{
			Con::cout<<"[PhysX] Debug Info in file '"<<file<<":"<<line<<"': "<<message<<Con::endl;
			break;
		}
		case physx::PxErrorCode::eDEBUG_WARNING:
		{
			Con::cwar<<"[PhysX] Warning in file '"<<file<<":"<<line<<"': "<<message<<Con::endl;
			break;
		}
		default:
		{
			Con::cerr<<"[PhysX] Error in file '"<<file<<":"<<line<<"': "<<message<<Con::endl;
			break;
		}
		}
	}
};

static PhysXErrorCallback gDefaultErrorCallback {};
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
static pragma::physics::PxUniquePtr<physx::PxPhysics> g_pxPhysics = pragma::physics::px_null_ptr<physx::PxPhysics>();
static pragma::physics::PxUniquePtr<physx::PxPvd> g_pxPvd = pragma::physics::px_null_ptr<physx::PxPvd>();
physx::PxFoundation &pragma::physics::PxEnvironment::GetFoundation() {return *g_pxFoundation;}
physx::PxPhysics &pragma::physics::PxEnvironment::GetPhysics() {return *g_pxPhysics;}
physx::PxPvd &pragma::physics::PxEnvironment::GetPVD() {return *g_pxPvd;}
extern "C"
{
	PRAGMA_EXPORT bool pragma_attach(std::string&)
	{
		if(g_instances++ > 0)
			return true;
		return true;
	}
	PRAGMA_EXPORT void pragma_detach()
	{
		if(--g_instances > 0 || g_pxFoundation == nullptr)
			return;
		PxCloseExtensions();
		physx::PxCloseVehicleSDK();
		g_pxPhysics = nullptr;
		g_pxPvd = nullptr;
		g_pxFoundation = nullptr;
	}
};

physx::PxFilterFlags testtt(
	physx::PxFilterObjectAttributes attributes0,
	physx::PxFilterData filterData0, 
	physx::PxFilterObjectAttributes attributes1,
	physx::PxFilterData filterData1,
	physx::PxPairFlags& pairFlags,
	const void* constantBlock,
	physx::PxU32 constantBlockSize)
{
	return physx::PxFilterFlag::eKILL;
}

class FilterCallback
	: public physx::PxSimulationFilterCallback
{
public:

	/**
	\brief Filter method to specify how a pair of potentially colliding objects should be processed.

	This method gets called when the filter flags returned by the filter shader (see #PxSimulationFilterShader)
	indicate that the filter callback should be invoked (#PxFilterFlag::eCALLBACK or #PxFilterFlag::eNOTIFY set).
	Return the PxFilterFlag flags and set the PxPairFlag flags to define what the simulation should do with the given 
	collision pair.

	\param[in] pairID Unique ID of the collision pair used to issue filter status changes for the pair (see #statusChange())
	\param[in] attributes0 The filter attribute of the first object
	\param[in] filterData0 The custom filter data of the first object
	\param[in] a0 Actor pointer of the first object
	\param[in] s0 Shape pointer of the first object (NULL if the object has no shapes)
	\param[in] attributes1 The filter attribute of the second object
	\param[in] filterData1 The custom filter data of the second object
	\param[in] a1 Actor pointer of the second object
	\param[in] s1 Shape pointer of the second object (NULL if the object has no shapes)
	\param[in,out] pairFlags In: Pair flags returned by the filter shader. Out: Additional information on how an accepted pair should get processed
	\return Filter flags defining whether the pair should be discarded, temporarily ignored or processed and whether the pair
	should be tracked and send a report on pair deletion through the filter callback

	@see PxSimulationFilterShader PxFilterData PxFilterObjectAttributes PxFilterFlag PxPairFlag
	*/
	virtual		physx::PxFilterFlags	pairFound(	physx::PxU32 pairID,
		physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0, const physx::PxActor* a0, const physx::PxShape* s0,
		physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1, const physx::PxActor* a1, const physx::PxShape* s1,
		physx::PxPairFlags& pairFlags) override
	{
		return physx::PxFilterFlag::eKILL;
	}

	/**
	\brief Callback to inform that a tracked collision pair is gone.

	This method gets called when a collision pair disappears or gets re-filtered. Only applies to
	collision pairs which have been marked as filter callback pairs (#PxFilterFlag::eNOTIFY set in #pairFound()).

	\param[in] pairID Unique ID of the collision pair that disappeared
	\param[in] attributes0 The filter attribute of the first object
	\param[in] filterData0 The custom filter data of the first object
	\param[in] attributes1 The filter attribute of the second object
	\param[in] filterData1 The custom filter data of the second object
	\param[in] objectRemoved True if the pair was lost because one of the objects got removed from the scene

	@see pairFound() PxSimulationFilterShader PxFilterData PxFilterObjectAttributes
	*/
	virtual		void			pairLost(	physx::PxU32 pairID,
		physx::PxFilterObjectAttributes attributes0,
		physx::PxFilterData filterData0,
		physx::PxFilterObjectAttributes attributes1,
		physx::PxFilterData filterData1,
		bool objectRemoved) override
	{

	}

	/**
	\brief Callback to give the opportunity to change the filter state of a tracked collision pair.

	This method gets called once per simulation step to let the application change the filter and pair
	flags of a collision pair that has been reported in #pairFound() and requested callbacks by
	setting #PxFilterFlag::eNOTIFY. To request a change of filter status, the target pair has to be
	specified by its ID, the new filter and pair flags have to be provided and the method should return true.

	\note If this method changes the filter status of a collision pair and the pair should keep being tracked
	by the filter callbacks then #PxFilterFlag::eNOTIFY has to be set.

	\note The application is responsible to ensure that this method does not get called for pairs that have been
	reported as lost, see #pairLost().

	\param[out] pairID ID of the collision pair for which the filter status should be changed
	\param[out] pairFlags The new pairFlags to apply to the collision pair
	\param[out] filterFlags The new filterFlags to apply to the collision pair
	\return True if the changes should be applied. In this case the method will get called again. False if
	no more status changes should be done in the current simulation step. In that case the provided flags will be discarded.

	@see pairFound() pairLost() PxFilterFlag PxPairFlag
	*/
	virtual		bool			statusChange(physx::PxU32& pairID, physx::PxPairFlags& pairFlags, physx::PxFilterFlags& filterFlags) override
	{
		return false;
	}

protected:
	virtual						~FilterCallback() override
	{

	}
};

bool pragma::physics::PxEnvironment::Initialize()
{
	if(g_pxFoundation == nullptr)
	{
#ifdef _WIN32
		PxSetPhysXDelayLoadHook(&g_DelayLoadHook);
		PxSetPhysXCookingDelayLoadHook(&g_DelayLoadHook);
		PxSetPhysXCommonDelayLoadHook(&g_DelayLoadHook);
#endif
		g_pxFoundation = {PxCreateFoundation(PX_PHYSICS_VERSION,gDefaultAllocatorCallback,gDefaultErrorCallback),[](physx::PxFoundation *pFoundation) {
			if(pFoundation)
				pFoundation->release();
		}};
	}

	if(g_pxFoundation == nullptr)
		return false;
	if(g_pxPvd == nullptr)
	{
		g_pxPvd = px_create_unique_ptr(physx::PxCreatePvd(*g_pxFoundation));
		if(g_pxPvd == nullptr)
			return false;
		auto *pTransport = physx::PxDefaultPvdSocketTransportCreate("127.0.0.1",5425,10);
		g_pxPvd->connect(*pTransport,physx::PxPvdInstrumentationFlag::eALL);
	}
	auto bEnableDebugging = g_pxPvd != nullptr;

	physx::PxTolerancesScale scale;
	scale.length = util::metres_to_units(1);
	scale.speed = 600.f;
	if(g_pxPhysics == nullptr)
	{
		g_pxPhysics = px_create_unique_ptr(PxCreatePhysics(PX_PHYSICS_VERSION,*g_pxFoundation,scale,bEnableDebugging,g_pxPvd.get()));
		if(g_pxPhysics == nullptr)
			return false;
	}

	m_cpuDispatcher = px_create_unique_ptr(physx::PxDefaultCpuDispatcherCreate(6)); // TODO: Should match number of available hardware threads
	if(m_cpuDispatcher == nullptr)
		return false;
	physx::PxSceneDesc sceneDesc {scale};
	sceneDesc.gravity = {0.f,0.f,0.f};
	sceneDesc.simulationEventCallback = nullptr;
	sceneDesc.filterShader = testtt;//physx::PxDefaultSimulationFilterShader;
	physx::PxSimulationFilterCallback;
	sceneDesc.filterCallback = new FilterCallback{};//nullptr;
	sceneDesc.kineKineFilteringMode = physx::PxPairFilteringMode::eDEFAULT;
	sceneDesc.staticKineFilteringMode = physx::PxPairFilteringMode::eDEFAULT;
	sceneDesc.broadPhaseType = physx::PxBroadPhaseType::eABP;
	sceneDesc.broadPhaseCallback = nullptr;
	sceneDesc.limits.setToDefault();
	sceneDesc.limits.maxNbActors = 131'072;
	sceneDesc.limits.maxNbAggregates = 256; // Unsure what this is for
	sceneDesc.limits.maxNbBodies = 32'768;
	sceneDesc.limits.maxNbBroadPhaseOverlaps = 256; // Unsure what this is for
	sceneDesc.limits.maxNbConstraints = 4'096;
	sceneDesc.limits.maxNbDynamicShapes = 16'384;
	sceneDesc.limits.maxNbRegions = 256; // Unsure what this is for
	sceneDesc.limits.maxNbStaticShapes = 65'536;
	sceneDesc.frictionType = physx::PxFrictionType::ePATCH;
	sceneDesc.solverType = physx::PxSolverType::ePGS;
	sceneDesc.cpuDispatcher = m_cpuDispatcher.get();

	// sceneDesc.bounceThresholdVelocity // TODO
	// sceneDesc.frictionOffsetThreshold; // TODO
	// sceneDesc.ccdMaxSeparation; // TODO
	sceneDesc.solverOffsetSlop = 0.0;
	sceneDesc.flags = physx::PxSceneFlag::eENABLE_CCD;

	m_scene = px_create_unique_ptr(g_pxPhysics->createScene(sceneDesc));
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
	m_controllerManager->setDebugRenderingFlags(physx::PxControllerDebugRenderFlag::eALL);
	if(PxInitExtensions(*g_pxPhysics,g_pxPvd.get()) == false || physx::PxInitVehicleSDK(*g_pxPhysics) == false)
		return false;
	physx::PxVehicleSetBasisVectors(ToPhysXNormal(uvec::UP),ToPhysXNormal(uvec::FORWARD));
	physx::PxVehicleSetUpdateMode(physx::PxVehicleUpdateMode::eVELOCITY_CHANGE);

	m_controllerBehaviorCallback = std::make_unique<CustomControllerBehaviorCallback>();
	m_controllerHitReport = std::make_unique<CustomUserControllerHitReport>();

	return IEnvironment::Initialize();
}
physx::PxVec3 pragma::physics::PxEnvironment::ToPhysXVector(const Vector3 &v) const
{
	return physx::PxVec3{v.x,v.y,v.z};
}
physx::PxExtendedVec3 pragma::physics::PxEnvironment::ToPhysXExtendedVector(const Vector3 &v) const
{
	return physx::PxExtendedVec3{v.x,v.y,v.z};
}
Vector3 pragma::physics::PxEnvironment::FromPhysXVector(const physx::PxExtendedVec3 &v) const
{
	return Vector3{static_cast<float>(v.x),static_cast<float>(v.y),static_cast<float>(v.z)};
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
physx::PxScene &pragma::physics::PxEnvironment::GetScene() const {return *m_scene;}
double pragma::physics::PxEnvironment::ToPhysXLength(double len) const {return len;}
double pragma::physics::PxEnvironment::FromPhysXLength(double len) const {return len;}

const Color &pragma::physics::PxEnvironment::FromPhysXColor(uint32_t color)
{
	switch(color)
	{
	case physx::PxDebugColor::eARGB_BLACK:
		return Color::Black;
	case physx::PxDebugColor::eARGB_RED:
		return Color::Red;
	case physx::PxDebugColor::eARGB_GREEN:
		return Color::Green;
	case physx::PxDebugColor::eARGB_BLUE:
		return Color::Blue;
	case physx::PxDebugColor::eARGB_YELLOW:
		return Color::Yellow;
	case physx::PxDebugColor::eARGB_MAGENTA:
		return Color::Magenta;
	case physx::PxDebugColor::eARGB_CYAN:
		return Color::Cyan;
	case physx::PxDebugColor::eARGB_WHITE:
		return Color::White;
	case physx::PxDebugColor::eARGB_GREY:
		return Color::LightGrey;
	case physx::PxDebugColor::eARGB_DARKRED:
		return Color::DarkRed;
	case physx::PxDebugColor::eARGB_DARKGREEN:
		return Color::DarkGreen;
	case physx::PxDebugColor::eARGB_DARKBLUE:
		return Color::DarkBlue;
	}
	return Color::White;
}

std::shared_ptr<pragma::physics::IMaterial> pragma::physics::PxEnvironment::CreateMaterial(float staticFriction,float dynamicFriction,float restitution)
{
	auto pMat = px_create_unique_ptr(g_pxPhysics->createMaterial(staticFriction,dynamicFriction,restitution));
	return CreateSharedPtr<PxMaterial>(*this,std::move(pMat));
}

pragma::physics::IEnvironment::RemainingDeltaTime pragma::physics::PxEnvironment::StepSimulation(float timeStep,int maxSubSteps,float fixedTimeStep)
{
	if(fixedTimeStep == 0.f)
		return timeStep;
	
	for(auto &hController : GetControllers())
		PxController::GetController(*hController).PreSimulate();

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

	for(auto &hController : GetControllers())
		PxController::GetController(*hController).PostSimulate();
	
	auto *pVisDebugger = GetVisualDebugger();
	if(pVisDebugger)
	{
		// TODO
		m_scene->setVisualizationParameter(physx::PxVisualizationParameter::eACTOR_AXES,1.f);
		m_scene->setVisualizationParameter(physx::PxVisualizationParameter::eBODY_AXES,1.f);
		m_scene->setVisualizationParameter(physx::PxVisualizationParameter::eCOLLISION_SHAPES,1.f);
		m_scene->setVisualizationParameter(physx::PxVisualizationParameter::eCOLLISION_DYNAMIC,1.f);
		m_scene->setVisualizationParameter(physx::PxVisualizationParameter::eCOLLISION_STATIC,1.f);
		m_scene->setVisualizationParameter(physx::PxVisualizationParameter::eCONTACT_POINT,1.f);
		m_scene->setVisualizationParameter(physx::PxVisualizationParameter::eCONTACT_NORMAL,1.f);
		m_scene->setVisualizationParameter(physx::PxVisualizationParameter::eWORLD_AXES,1.f);

		m_scene->setVisualizationParameter(physx::PxVisualizationParameter::eSCALE,1.f);

		auto &renderBuffer = m_scene->getRenderBuffer();
		pVisDebugger->Reset();

		auto numLines = renderBuffer.getNbLines();
		auto *pLines = renderBuffer.getLines();
		for(auto i=decltype(numLines){0u};i<numLines;++i)
		{
			auto &line = pLines[i];
			pVisDebugger->DrawLine(FromPhysXVector(line.pos0),FromPhysXVector(line.pos1),FromPhysXColor(line.color0),FromPhysXColor(line.color1));
		}

		auto numPoints = renderBuffer.getNbPoints();
		auto *pPoints = renderBuffer.getPoints();
		for(auto i=decltype(numPoints){0u};i<numPoints;++i)
		{
			auto &point = pPoints[i];
			pVisDebugger->DrawPoint(FromPhysXVector(point.pos),FromPhysXColor(point.color));
		}

		auto numTris = renderBuffer.getNbTriangles();
		auto *pTris = renderBuffer.getTriangles();
		for(auto i=decltype(numTris){0u};i<numTris;++i)
		{
			auto &tri = pTris[i];
			pVisDebugger->DrawTriangle(FromPhysXVector(tri.pos0),FromPhysXVector(tri.pos1),FromPhysXVector(tri.pos2),FromPhysXColor(tri.color0),FromPhysXColor(tri.color1),FromPhysXColor(tri.color2));
		}

		auto numTexts = renderBuffer.getNbTexts();
		auto *pTexts = renderBuffer.getTexts();
		for(auto i=decltype(numTexts){0u};i<numTexts;++i)
		{
			auto &text = pTexts[i];
			pVisDebugger->DrawText(text.string,FromPhysXVector(text.position),FromPhysXColor(text.color),text.size);
		}
		pVisDebugger->Flush();
	}
	return fmodf(timeStep,fixedTimeStep);
}
#pragma optimize("",on)
