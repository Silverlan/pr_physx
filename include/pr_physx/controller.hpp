/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

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
	class PhysXEnvironment;
	class PhysXQueryFilterCallback;
	class CustomUserControllerHitReport;
	class IMaterial;
	class PhysXController
		: virtual public IController
	{
	public:
		friend IEnvironment;
		friend PhysXEnvironment;
		friend CustomUserControllerHitReport;
		static PhysXController &GetController(IController &c);
		static const PhysXController &GetController(const IController &c);
		virtual ~PhysXController() override;
		physx::PxController &GetInternalObject() const;
		PhysXEnvironment &GetPxEnv() const;

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

		virtual Vector3 GetLinearVelocity() const override;
		virtual void SetLinearVelocity(const Vector3 &vel) override;

		void MoveController(const Vector3 &displacement,bool testOnly);
	protected:
		PhysXController(IEnvironment &env,PhysXUniquePtr<physx::PxController> controller,const util::TSharedHandle<ICollisionObject> &collisionObject);
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
		void PreSimulate(float dt);
		void PostSimulate(float dt);
		Vector3 m_velocity {};
		Vector3 m_preSimulationPosition = {};
		PhysXUniquePtr<physx::PxController> m_controller = px_null_ptr<physx::PxController>();
		std::unique_ptr<PhysXQueryFilterCallback> m_queryFilterCallback = nullptr;
		physx::PxControllerState m_controllerState;
		pragma::physics::IController::CollisionFlags m_collisionFlags = pragma::physics::IController::CollisionFlags::None;
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
