#ifndef __PR_PX_VEHICLE_HPP__
#define __PR_PX_VEHICLE_HPP__

#include "pr_physx/common.hpp"
#include <pragma/physics/vehicle.hpp>

namespace pragma::physics
{
	class ICollisionObject;
	class PhysXEnvironment;
	class PhysXQueryFilterCallback;
	class PhysXVehicleDrive;
	class PhysXVehicle
		: public IVehicle
	{
	public:
		friend PhysXEnvironment;
		friend IEnvironment;
		static PhysXVehicle &GetVehicle(IVehicle &v);
		static const PhysXVehicle &GetVehicle(const IVehicle &v);
		physx::PxVehicleDrive &GetInternalObject() const;
		PhysXEnvironment &GetPxEnv() const;

		uint32_t GetWheelCount() const;
		float GetForwardSpeed() const;
		float GetSidewaysSpeed() const;
	private:
		PhysXVehicle(IEnvironment &env,PhysXUniquePtr<physx::PxVehicleDrive> vhc,const util::TSharedHandle<ICollisionObject> &collisionObject);
		virtual void Initialize() override;
		virtual void RemoveWorldObject() override;
		virtual void DoAddWorldObject() override;
		void Simulate(float dt);
		PhysXUniquePtr<physx::PxVehicleDrive> m_vehicle = px_null_ptr<physx::PxVehicleDrive>();
		PhysXUniquePtr<physx::PxBatchQuery> m_raycastBatchQuery = px_null_ptr<physx::PxBatchQuery>();

		std::vector<physx::PxRaycastQueryResult> m_raycastQueryResultPerWheel;
		std::vector<physx::PxWheelQueryResult> m_wheelQueryResults;
		std::vector<physx::PxRaycastHit> m_raycastHitPerWheel;
	};

	class PhysXWheel
		: public IWheel
	{
	public:
		static PhysXWheel &GetWheel(IWheel &w);
		static const PhysXWheel &GetWheel(const IWheel &w);
	private:
		PhysXWheel(IEnvironment &env);
		virtual void Initialize() override;
		PhysXEnvironment &GetPxEnv() const;
	};
};

#endif
