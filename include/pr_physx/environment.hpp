#ifndef __PR_PX_ENVIRONMENT_HPP__
#define __PR_PX_ENVIRONMENT_HPP__

#include <pragma/physics/environment.hpp>
#include <mathutil/uvec.h>
#include "pr_physx/common.hpp"

namespace physx
{
	class PxFoundation;
	class PxPhysics;
	class PxCooking;
	class PxPvd;
	class PxScene;
	class PxVec3;
	class PxTransform;
	class PxRaycastHit;
	class PxOverlapHit;
	class PxSweepHit;
	class PxRigidActor;
	class PxControllerDesc;
	class PxDefaultCpuDispatcher;
};
class NetworkState;
enum class RayCastHitType : uint8_t;
namespace pragma::physics
{
	class CustomControllerBehaviorCallback;
	class CustomUserControllerHitReport;
	class PxRigidBody;
	class PxCollisionObject;
	class PxController;
	class PxMaterial;
	class PxShape;
	class PxTriangleShape;
	class PxConvexHullShape;
	struct WheelCreateInfo;
	struct ChassisCreateInfo;
	class PxEnvironment
		: public pragma::physics::IEnvironment
	{
	public:
		PxEnvironment(NetworkState &state);
		~PxEnvironment();
		static Transform CreateTransform(const physx::PxTransform &pxTransform);
		static physx::PxTransform CreatePxTransform(const Transform &btTransform);
		static PxCollisionObject *GetCollisionObject(const physx::PxRigidActor &actor);
		static PxShape *GetShape(const physx::PxShape &shape);
		static PxController *GetController(const physx::PxController &controller);
		static PxMaterial *GetMaterial(const physx::PxMaterial &material);
		static physx::PxFoundation &GetFoundation();
		static physx::PxPhysics &GetPhysics();
		static physx::PxPvd &GetPVD();

		virtual bool Initialize() override;

		physx::PxVec3 ToPhysXVector(const Vector3 &v) const;
		physx::PxExtendedVec3 ToPhysXExtendedVector(const Vector3 &v) const;
		physx::PxVec3 ToPhysXNormal(const Vector3 &n) const;
		physx::PxVec3 ToPhysXTorque(const Vector3 &t) const;
		physx::PxQuat ToPhysXRotation(const Quat &rot) const;
		Vector3 FromPhysXVector(const physx::PxVec3 &v) const;
		Vector3 FromPhysXVector(const physx::PxExtendedVec3 &v) const;
		Vector3 FromPhysXNormal(const physx::PxVec3 &n) const;
		Vector3 FromPhysXTorque(const physx::PxVec3 &t) const;
		Quat FromPhysXRotation(const physx::PxQuat &v) const;
		double ToPhysXLength(double len) const;
		double FromPhysXLength(double len) const;
		static const Color &FromPhysXColor(uint32_t color);

		PxRigidBody &ToBtType(IRigidBody &body);

		virtual util::TSharedHandle<IFixedConstraint> CreateFixedConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB) override;
		virtual util::TSharedHandle<IBallSocketConstraint> CreateBallSocketConstraint(IRigidBody &a,const Vector3 &pivotA,IRigidBody &b,const Vector3 &pivotB) override;
		virtual util::TSharedHandle<IHingeConstraint> CreateHingeConstraint(IRigidBody &a,const Vector3 &pivotA,IRigidBody &b,const Vector3 &pivotB,const Vector3 &axis) override;
		virtual util::TSharedHandle<ISliderConstraint> CreateSliderConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB) override;
		virtual util::TSharedHandle<IConeTwistConstraint> CreateConeTwistConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB) override;
		virtual util::TSharedHandle<IDoFConstraint> CreateDoFConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB) override;
		virtual util::TSharedHandle<IDoFSpringConstraint> CreateDoFSpringConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB) override;

		virtual util::TSharedHandle<IController> CreateCapsuleController(float halfWidth,float halfHeight,float stepHeight,umath::Degree slopeLimit=DEFAULT_CHARACTER_SLOPE_LIMIT,const Transform &startTransform={}) override;
		virtual util::TSharedHandle<IController> CreateBoxController(const Vector3 &halfExtents,float stepHeight,umath::Degree slopeLimit=DEFAULT_CHARACTER_SLOPE_LIMIT,const Transform &startTransform={}) override;
		virtual util::TSharedHandle<ICollisionObject> CreateCollisionObject(IShape &shape) override;
		virtual util::TSharedHandle<IRigidBody> CreateRigidBody(float mass,IShape &shape,const Vector3 &localInertia,bool dynamic) override;
		virtual util::TSharedHandle<ISoftBody> CreateSoftBody(const PhysSoftBodyInfo &info,float mass,const std::vector<Vector3> &verts,const std::vector<uint16_t> &indices,std::vector<uint16_t> &indexTranslations) override;
		virtual util::TSharedHandle<IGhostObject> CreateGhostObject(IShape &shape) override;

		virtual std::shared_ptr<IConvexShape> CreateCapsuleShape(float halfWidth,float halfHeight,const IMaterial &mat) override;
		virtual std::shared_ptr<IConvexShape> CreateBoxShape(const Vector3 &halfExtents,const IMaterial &mat) override;
		virtual std::shared_ptr<IConvexShape> CreateCylinderShape(float radius,float height,const IMaterial &mat) override;
		virtual std::shared_ptr<IConvexShape> CreateSphereShape(float radius,const IMaterial &mat) override;
		virtual std::shared_ptr<IConvexHullShape> CreateConvexHullShape(const IMaterial &mat) override;
		virtual std::shared_ptr<ITriangleShape> CreateTriangleShape(const IMaterial &mat) override;
		virtual std::shared_ptr<ICompoundShape> CreateCompoundShape(std::vector<IShape*> &shapes) override;
		virtual std::shared_ptr<IShape> CreateHeightfieldTerrainShape(uint32_t width,uint32_t length,Scalar maxHeight,uint32_t upAxis,const IMaterial &mat) override;
		virtual std::shared_ptr<IMaterial> CreateMaterial(float staticFriction,float dynamicFriction,float restitution) override;

		void CreateVehicle();
		physx::PxScene &GetScene() const;

		virtual RemainingDeltaTime StepSimulation(float timeStep,int maxSubSteps=1,float fixedTimeStep=(1.f /60.f)) override;

		virtual Bool Overlap(const TraceData &data,std::vector<TraceResult> *optOutResults=nullptr) const override;
		virtual Bool RayCast(const TraceData &data,std::vector<TraceResult> *optOutResults=nullptr) const override;
		virtual Bool Sweep(const TraceData &data,std::vector<TraceResult> *optOutResults=nullptr) const override;

		physx::PxCooking &GetCooking();

		template<class T,typename... TARGS>
			PxUniquePtr<T> CreateUniquePtr(TARGS&& ...args);
	private:
		friend PxTriangleShape;
		friend PxConvexHullShape;
		util::TSharedHandle<IController> CreateController(PxUniquePtr<physx::PxController> controller);
		void CreateWheel(const WheelCreateInfo &createInfo);
		void InitializeShape(PxShape &shape,bool basicOnly=false);
		void InitializeCollisionObject(PxCollisionObject &o);
		void InitializeRayCastResult(const TraceData &data,float rayLength,const physx::PxRaycastHit &raycastHit,TraceResult &outResult,RayCastHitType hitType) const;
		void InitializeRayCastResult(const TraceData &data,float rayLength,const physx::PxOverlapHit &raycastHit,TraceResult &outResult,RayCastHitType hitType) const;
		void InitializeRayCastResult(const TraceData &data,float rayLength,const physx::PxSweepHit &raycastHit,TraceResult &outResult,RayCastHitType hitType) const;
		void InitializeControllerDesc(physx::PxControllerDesc &inOutDesc,float stepHeight,const Transform &startTransform);

		PxUniquePtr<physx::PxCooking> m_cooking = px_null_ptr<physx::PxCooking>();
		PxUniquePtr<physx::PxScene> m_scene = px_null_ptr<physx::PxScene>();
		PxUniquePtr<physx::PxControllerManager> m_controllerManager = px_null_ptr<physx::PxControllerManager>();
		PxUniquePtr<physx::PxDefaultCpuDispatcher> m_cpuDispatcher = px_null_ptr<physx::PxDefaultCpuDispatcher>();

		std::unique_ptr<CustomControllerBehaviorCallback> m_controllerBehaviorCallback = nullptr;
		std::unique_ptr<CustomUserControllerHitReport> m_controllerHitReport = nullptr;
	};
};

template<class T,typename... TARGS>
	pragma::physics::PxUniquePtr<T> pragma::physics::PxEnvironment::CreateUniquePtr(TARGS&& ...args)
{
	return std::unique_ptr<T,void(*)(T *t)>{new T{std::forward<TARGS>(args)...},[](T *t) {
		t->OnRemove();
		delete t;
	}};
}

#endif
