/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include <cinttypes>
#include <limits>
#include <pragma/entities/entity_component_manager.hpp>
#include "pr_module.hpp"
#include "pr_physx/environment.hpp"
#include "pr_physx/shape.hpp"
#include "pr_physx/collision_object.hpp"
#include "pr_physx/constraint.hpp"
#include "pr_physx/controller.hpp"
#include "pr_physx/material.hpp"
#include "pr_physx/raycast.hpp"
#include "pr_physx/vehicle.hpp"
#include "pr_physx/sim_event_callback.hpp"
#include "pr_physx/sim_filter_shader.hpp"
#include <sharedutils/util.h>
#include <sharedutils/util_pragma.hpp>
#include <pragma/math/surfacematerial.h>
#include <mathutil/transform.hpp>
#include <pragma/physics/visual_debugger.hpp>
#include <pragma/entities/baseentity.h>
#include <pragma/util/util_game.hpp>
#include <pragma/networkstate/networkstate.h>
#include <PxPhysicsAPI.h>
#ifdef _WIN32
#include <common/windows/PxWindowsDelayLoadHook.h>
#endif

umath::Transform pragma::physics::PhysXEnvironment::CreateTransform(const physx::PxTransform &pxTransform) { return umath::Transform {uvec::create(pxTransform.p), uquat::create(pxTransform.q)}; }
physx::PxTransform pragma::physics::PhysXEnvironment::CreatePxTransform(const umath::Transform &t) { return physx::PxTransform {uvec::create_px(t.GetOrigin()), uquat::create_px(t.GetRotation())}; }
pragma::physics::PhysXCollisionObject *pragma::physics::PhysXEnvironment::GetCollisionObject(const physx::PxActor &actor) { return static_cast<pragma::physics::PhysXCollisionObject *>(actor.userData); }
pragma::physics::PhysXConstraint *pragma::physics::PhysXEnvironment::GetConstraint(const physx::PxJoint &constraint) { return static_cast<pragma::physics::PhysXConstraint *>(constraint.userData); }
pragma::physics::PhysXActorShape *pragma::physics::PhysXEnvironment::GetShape(const physx::PxShape &shape) { return static_cast<pragma::physics::PhysXActorShape *>(shape.userData); }
pragma::physics::PhysXController *pragma::physics::PhysXEnvironment::GetController(const physx::PxController &controller) { return static_cast<pragma::physics::PhysXController *>(controller.getUserData()); }
pragma::physics::PhysXMaterial *pragma::physics::PhysXEnvironment::GetMaterial(const physx::PxBaseMaterial &material) { return static_cast<pragma::physics::PhysXMaterial *>(material.userData); }
pragma::physics::PhysXEnvironment::PhysXEnvironment(NetworkState &state) : IEnvironment {state} {}

void pragma::physics::PhysXEnvironment::OnRemove()
{
	IEnvironment::OnRemove();
	m_controllerManager = nullptr;
	m_scene = nullptr;
	m_cpuDispatcher = nullptr;
	m_surfaceTirePairs = nullptr;
	m_controllerBehaviorCallback = nullptr;
	m_controllerHitReport = nullptr;
	m_simEventCallback = nullptr;
	m_simFilterCallback = nullptr;
}

class PhysXErrorCallback : public physx::PxErrorCallback {
  public:
	virtual void reportError(physx::PxErrorCode::Enum code, const char *message, const char *file, int line) override
	{
		switch(code) {
		case physx::PxErrorCode::eDEBUG_INFO:
			{
				Con::cout << "[PhysX] Debug Info in file '" << file << ":" << line << "': " << message << Con::endl;
				break;
			}
		case physx::PxErrorCode::eDEBUG_WARNING:
			{
				Con::cwar << "[PhysX] Warning in file '" << file << ":" << line << "': " << message << Con::endl;
				break;
			}
		default:
			{
				Con::cerr << "[PhysX] Error in file '" << file << ":" << line << "': " << message << Con::endl;
				break;
			}
		}
	}
};

static PhysXErrorCallback gDefaultErrorCallback {};
static physx::PxDefaultAllocator gDefaultAllocatorCallback {};

extern "C" {
PRAGMA_EXPORT void initialize_physics_engine(NetworkState &nw, std::unique_ptr<pragma::physics::IEnvironment> &outEnv);
};

#ifdef _WIN32
class PxDelayLoadHook : public physx::PxDelayLoadHook {
	std::string get_library_path(const std::string &libName) const
	{
		auto path = util::get_path_to_library(initialize_physics_engine);
		if(path.has_value() == false)
			return libName;
		return *path + '/' + libName;
	}
	virtual const char *getPhysXCommonDllName() const { return get_library_path("PhysXCommon_64.dll").c_str(); }

	virtual const char *getPhysXFoundationDllName() const { return get_library_path("PhysXFoundation_64.dll").c_str(); }
} static g_DelayLoadHook;
#endif

static uint32_t g_instances = 0;
static pragma::physics::PhysXUniquePtr<physx::PxFoundation> g_pxFoundation = pragma::physics::px_null_ptr<physx::PxFoundation>();
static pragma::physics::PhysXUniquePtr<physx::PxPhysics> g_pxPhysics = pragma::physics::px_null_ptr<physx::PxPhysics>();
static pragma::physics::PhysXUniquePtr<physx::PxPvd> g_pxPvd = pragma::physics::px_null_ptr<physx::PxPvd>();
physx::PxFoundation &pragma::physics::PhysXEnvironment::GetFoundation() { return *g_pxFoundation; }
physx::PxPhysics &pragma::physics::PhysXEnvironment::GetPhysics() { return *g_pxPhysics; }
physx::PxPvd &pragma::physics::PhysXEnvironment::GetPVD() { return *g_pxPvd; }
extern "C" {
PRAGMA_EXPORT bool pragma_attach(std::string &)
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
/*
physx::PxFilterFlags VehicleFilterShader
(physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0, 
	physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1,
	physx::PxPairFlags& pairFlags, const void* constantBlock, physx::PxU32 constantBlockSize)
{
	PX_UNUSED(attributes0);
	PX_UNUSED(attributes1);
	PX_UNUSED(constantBlock);
	PX_UNUSED(constantBlockSize);

	if(PX_FILTER_SHOULD_PASS(filterData0,filterData1) == false)
		return physx::PxFilterFlag::eSUPPRESS;

	pairFlags = physx::PxPairFlag::eCONTACT_DEFAULT;
	pairFlags |= physx::PxPairFlags(physx::PxU16(filterData0.word2 | filterData1.word2));

	return physx::PxFilterFlags();
}
*/
bool pragma::physics::PhysXEnvironment::Initialize()
{
	if(g_pxFoundation == nullptr) {
#ifdef _WIN32
		PxSetPhysXDelayLoadHook(&g_DelayLoadHook);
		PxSetPhysXCookingDelayLoadHook(&g_DelayLoadHook);
		PxSetPhysXCommonDelayLoadHook(&g_DelayLoadHook);
#endif
		g_pxFoundation = {PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback), [](physx::PxFoundation *pFoundation) {
			                  if(pFoundation)
				                  pFoundation->release();
		                  }};
	}

	if(g_pxFoundation == nullptr)
		return false;
	if(g_pxPvd == nullptr) {
		g_pxPvd = px_create_unique_ptr(physx::PxCreatePvd(*g_pxFoundation));
		if(g_pxPvd == nullptr)
			return false;
		auto *pTransport = physx::PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
		g_pxPvd->connect(*pTransport, physx::PxPvdInstrumentationFlag::eALL);
	}
	auto bEnableDebugging = g_pxPvd != nullptr;

	physx::PxTolerancesScale scale;
	scale.length = util::pragma::metres_to_units(1);
	scale.speed = 600.f;
	if(g_pxPhysics == nullptr) {
		g_pxPhysics = px_create_unique_ptr(PxCreatePhysics(PX_PHYSICS_VERSION, *g_pxFoundation, scale, bEnableDebugging, g_pxPvd.get()));
		if(g_pxPhysics == nullptr)
			return false;
		if(PxInitExtensions(*g_pxPhysics, g_pxPvd.get()) == false || physx::PxInitVehicleSDK(*g_pxPhysics) == false)
			return false;
		// TODO
		physx::PxVehicleSetBasisVectors(ToPhysXNormal(uvec::UP), ToPhysXNormal(Vector3 {0.f, 0.f, 1.f})); //uvec::FORWARD));
		physx::PxVehicleSetUpdateMode(physx::PxVehicleUpdateMode::eVELOCITY_CHANGE);
	}

	m_cpuDispatcher = px_create_unique_ptr(physx::PxDefaultCpuDispatcherCreate(6)); // TODO: Should match number of available hardware threads
	if(m_cpuDispatcher == nullptr)
		return false;
	m_simEventCallback = std::make_unique<PhysXSimulationEventCallback>();
	m_simFilterCallback = std::make_unique<PhysXSimulationFilterCallback>();
	physx::PxSceneDesc sceneDesc {scale};
	sceneDesc.gravity = {0.f, 0.f, 0.f};
	sceneDesc.simulationEventCallback = m_simEventCallback.get();
	sceneDesc.filterCallback = m_simFilterCallback.get();
	sceneDesc.filterShader = PhysXSimulationFilterShader;
	//sceneDesc.filterShader = VehicleFilterShader;
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
	// sceneDesc.solverOffsetSlop = 0.0;
	sceneDesc.flags = physx::PxSceneFlag::eENABLE_CCD;

	m_scene = px_create_unique_ptr(g_pxPhysics->createScene(sceneDesc));
	if(m_scene == nullptr)
		return false;

	m_controllerManager = px_create_unique_ptr(PxCreateControllerManager(*m_scene));
	if(m_controllerManager == nullptr)
		return false;
	m_controllerManager->setOverlapRecoveryModule(true);
	m_controllerManager->setDebugRenderingFlags(physx::PxControllerDebugRenderFlag::eALL);

	m_controllerBehaviorCallback = std::make_unique<CustomControllerBehaviorCallback>();
	m_controllerHitReport = std::make_unique<CustomUserControllerHitReport>();
	return IEnvironment::Initialize();
}
pragma::physics::PhysXUniquePtr<pragma::physics::NoCollisionCategory> pragma::physics::PhysXEnvironment::GetUniqueNoCollisionCategory()
{
	NoCollisionCategoryId catId;
	if(m_freeNoCollisionCategories.empty() == false) {
		catId = m_freeNoCollisionCategories.front();
		m_freeNoCollisionCategories.pop();
	}
	else
		catId = m_nextNoCollisionCategoryId++;
	PhysXUniquePtr<NoCollisionCategory> cat {new NoCollisionCategory {*this, catId}, [](NoCollisionCategory *cat) {
		                                         cat->physEnv.m_freeNoCollisionCategories.push(static_cast<NoCollisionCategoryId>(*cat));
		                                         delete cat;
	                                         }};
	return cat;
}
void pragma::physics::PhysXEnvironment::StartProfiling() {}
void pragma::physics::PhysXEnvironment::EndProfiling() {}
physx::PxVec3 pragma::physics::PhysXEnvironment::ToPhysXVector(const Vector3 &v) const { return physx::PxVec3 {v.x, v.y, v.z}; }
physx::PxExtendedVec3 pragma::physics::PhysXEnvironment::ToPhysXExtendedVector(const Vector3 &v) const { return physx::PxExtendedVec3 {v.x, v.y, v.z}; }
Vector3 pragma::physics::PhysXEnvironment::FromPhysXVector(const physx::PxExtendedVec3 &v) const { return Vector3 {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)}; }
physx::PxVec3 pragma::physics::PhysXEnvironment::ToPhysXNormal(const Vector3 &n) const { return physx::PxVec3 {n.x, n.y, n.z}; }
physx::PxVec3 pragma::physics::PhysXEnvironment::ToPhysXTorque(const Vector3 &t) const { return physx::PxVec3 {t.x, t.y, t.z}; }
float pragma::physics::PhysXEnvironment::ToPhysXTorque(float force) const { return force; }
physx::PxQuat pragma::physics::PhysXEnvironment::ToPhysXRotation(const Quat &rot) const { return physx::PxQuat {rot.x, rot.y, rot.z, rot.w}; }
Vector3 pragma::physics::PhysXEnvironment::FromPhysXVector(const physx::PxVec3 &v) const { return Vector3 {v.x, v.y, v.z}; }
Vector3 pragma::physics::PhysXEnvironment::FromPhysXNormal(const physx::PxVec3 &n) const { return Vector3 {n.x, n.y, n.z}; }
Vector3 pragma::physics::PhysXEnvironment::FromPhysXTorque(const physx::PxVec3 &t) const { return Vector3 {t.x, t.y, t.z}; }
float pragma::physics::PhysXEnvironment::FromPhysXTorque(float force) const { return force; }
Quat pragma::physics::PhysXEnvironment::FromPhysXRotation(const physx::PxQuat &v) const { return Quat {v.w, v.x, v.y, v.z}; }
pragma::physics::PhysXRigidBody &pragma::physics::PhysXEnvironment::ToBtType(IRigidBody &body) { return dynamic_cast<PhysXRigidBody &>(body); }
physx::PxVehicleDrivableSurfaceToTireFrictionPairs &pragma::physics::PhysXEnvironment::GetVehicleSurfaceTireFrictionPairs() const { return *m_surfaceTirePairs; }
physx::PxScene &pragma::physics::PhysXEnvironment::GetScene() const { return *m_scene; }
double pragma::physics::PhysXEnvironment::ToPhysXLength(double len) const { return len; }
double pragma::physics::PhysXEnvironment::FromPhysXLength(double len) const { return len; }
float pragma::physics::PhysXEnvironment::FromPhysXMass(float mass) const { return mass * umath::pow3(util::pragma::units_to_metres(1.f)); }

const Color &pragma::physics::PhysXEnvironment::FromPhysXColor(uint32_t color)
{
	switch(color) {
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

std::shared_ptr<pragma::physics::IMaterial> pragma::physics::PhysXEnvironment::CreateMaterial(float staticFriction, float dynamicFriction, float restitution)
{
	auto pMat = px_create_unique_ptr(g_pxPhysics->createMaterial(staticFriction, dynamicFriction, restitution));
	return CreateSharedPtr<PhysXMaterial>(*this, std::move(pMat));
}

pragma::physics::IEnvironment::RemainingDeltaTime pragma::physics::PhysXEnvironment::DoStepSimulation(float timeStep, int maxSubSteps, float fixedTimeStep)
{
	if(fixedTimeStep == 0.f)
		return timeStep;

	for(auto &hController : GetControllers())
		PhysXController::GetController(*hController).PreSimulate(timeStep);

	auto t = timeStep / fixedTimeStep;
	auto numSubSteps = umath::floor(t);
	for(auto i = decltype(numSubSteps) {0u}; i < numSubSteps; ++i) {
		for(auto &vhc : GetVehicles())
			PhysXVehicle::GetVehicle(*vhc).Simulate(fixedTimeStep);

		m_scene->simulate(fixedTimeStep);
		physx::PxU32 err;
		auto success = m_scene->fetchResults(true, &err);
		if(err)
			;
	}

	for(auto &hController : GetControllers())
		PhysXController::GetController(*hController).PostSimulate(timeStep);

	auto *pVisDebugger = GetVisualDebugger();
	if(pVisDebugger) {
		m_scene->setVisualizationParameter(physx::PxVisualizationParameter::eACTOR_AXES, 1.f);
		m_scene->setVisualizationParameter(physx::PxVisualizationParameter::eBODY_AXES, 1.f);
		m_scene->setVisualizationParameter(physx::PxVisualizationParameter::eCOLLISION_SHAPES, 1.f);
		m_scene->setVisualizationParameter(physx::PxVisualizationParameter::eCONTACT_POINT, 1.f);
		m_scene->setVisualizationParameter(physx::PxVisualizationParameter::eCONTACT_NORMAL, 1.f);
		m_scene->setVisualizationParameter(physx::PxVisualizationParameter::eWORLD_AXES, 1.f);

		m_scene->setVisualizationParameter(physx::PxVisualizationParameter::eSCALE, 1.f);

		auto &renderBuffer = m_scene->getRenderBuffer();
		pVisDebugger->Reset();

		auto numLines = renderBuffer.getNbLines();
		auto *pLines = renderBuffer.getLines();
		for(auto i = decltype(numLines) {0u}; i < numLines; ++i) {
			auto &line = pLines[i];
			pVisDebugger->DrawLine(FromPhysXVector(line.pos0), FromPhysXVector(line.pos1), FromPhysXColor(line.color0), FromPhysXColor(line.color1));
		}

		auto numPoints = renderBuffer.getNbPoints();
		auto *pPoints = renderBuffer.getPoints();
		for(auto i = decltype(numPoints) {0u}; i < numPoints; ++i) {
			auto &point = pPoints[i];
			pVisDebugger->DrawPoint(FromPhysXVector(point.pos), FromPhysXColor(point.color));
		}

		auto numTris = renderBuffer.getNbTriangles();
		auto *pTris = renderBuffer.getTriangles();
		for(auto i = decltype(numTris) {0u}; i < numTris; ++i) {
			auto &tri = pTris[i];
			pVisDebugger->DrawTriangle(FromPhysXVector(tri.pos0), FromPhysXVector(tri.pos1), FromPhysXVector(tri.pos2), FromPhysXColor(tri.color0), FromPhysXColor(tri.color1), FromPhysXColor(tri.color2));
		}

		/*auto numTexts = renderBuffer.getNbTexts();
		auto *pTexts = renderBuffer.getTexts();
		for(auto i = decltype(numTexts) {0u}; i < numTexts; ++i) {
			auto &text = pTexts[i];
			pVisDebugger->DrawText(text.string, FromPhysXVector(text.position), FromPhysXColor(text.color), text.size);
		}*/
		pVisDebugger->Flush();
	}
	return fmodf(timeStep, fixedTimeStep);
}
