#ifndef __PR_PX_CONTROLLER_HPP__
#define __PR_PX_CONTROLLER_HPP__

#include "common.hpp"
#include <pragma/physics/controller.hpp>
#include <mathutil/uvec.h>

namespace physx
{
	class PxController;
};
namespace pragma::physics
{
	class PxEnvironment;
	class CustomUserControllerHitReport;
	class IMaterial;
	class PxController
		: virtual public IController
	{
	public:
		friend IEnvironment;
		friend PxEnvironment;
		friend CustomUserControllerHitReport;
		static PxController &GetController(IController &c);
		static const PxController &GetController(const IController &c);
		physx::PxController &GetInternalObject() const;
		PxEnvironment &GetPxEnv() const;

		virtual IShape *GetGroundShape() const override;
		virtual IRigidBody *GetGroundBody() const override;
		virtual IMaterial *GetGroundMaterial() const override;
		virtual bool IsTouchingGround() const override;
		virtual std::optional<Vector3> GetGroundTouchPos() const override;
		virtual std::optional<Vector3> GetGroundTouchNormal() const override;
		virtual CollisionFlags GetCollisionFlags() const override;
		virtual void DoMove(Vector3 &disp) override;
		virtual Vector3 GetDimensions() const override;
		virtual void SetDimensions(const Vector3 &dimensions) override;
		virtual void Resize(float newHeight) override;
		virtual void SetPos(const Vector3 &pos) override;
		virtual Vector3 GetPos() const override;
		virtual void SetFootPos(const Vector3 &footPos) override;
		virtual Vector3 GetFootPos() const override;
		virtual void SetUpDirection(const Vector3 &up) override;
		virtual Vector3 GetUpDirection() const override;
		virtual void SetSlopeLimit(umath::Degree slopeLimit) override;
		virtual umath::Degree GetSlopeLimit() const override;
		virtual void SetStepHeight(float stepHeight) override;
		virtual float GetStepHeight() const override;
	protected:
		PxController(IEnvironment &env,PxUniquePtr<physx::PxController> controller,const util::TSharedHandle<ICollisionObject> &collisionObject);
		virtual void Initialize() override;
		virtual void RemoveWorldObject() override;
		virtual void DoAddWorldObject() override;
		struct TouchingHit
		{
			std::weak_ptr<pragma::physics::IShape> shape;
			std::weak_ptr<pragma::physics::IMaterial> material;
			util::TWeakSharedHandle<pragma::physics::IRigidBody> body;
			Vector3 worldNormal;
			Vector3 worldPos;
		};
		void PreSimulate();
		void PostSimulate();
		PxUniquePtr<physx::PxController> m_controller = px_null_ptr<physx::PxController>();
		physx::PxControllerState m_controllerState;
		std::vector<TouchingHit> m_touchingHits = {};
		TouchingHit *m_pGroundTouchingHit = nullptr;
	};

	//////////////////////

	class CustomControllerBehaviorCallback
		: public physx::PxControllerBehaviorCallback
	{
	public:
		virtual physx::PxControllerBehaviorFlags getBehaviorFlags(const physx::PxShape& shape, const physx::PxActor& actor) override;
		virtual physx::PxControllerBehaviorFlags getBehaviorFlags(const physx::PxController& controller) override;
		virtual physx::PxControllerBehaviorFlags getBehaviorFlags(const physx::PxObstacle& obstacle) override;
	};

	//////////////////////

	class CustomUserControllerHitReport
		: public physx::PxUserControllerHitReport
	{
	public:
		virtual void onShapeHit(const physx::PxControllerShapeHit& hit) override;
		virtual void onControllerHit(const physx::PxControllersHit& hit) override;
		virtual void onObstacleHit(const physx::PxControllerObstacleHit& hit) override;
	};
};

#endif
