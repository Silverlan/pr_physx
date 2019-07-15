#ifndef __PR_PX_VEHICLE_HPP__
#define __PR_PX_VEHICLE_HPP__

#include "pr_physx/common.hpp"
#include <pragma/physics/vehicle.hpp>

namespace snippetvehicle
{

	using namespace physx;

	enum
	{
		DRIVABLE_SURFACE = 0xffff0000,
		UNDRIVABLE_SURFACE = 0x0000ffff
	};

	void setupDrivableSurface(PxFilterData& filterData);

	void setupNonDrivableSurface(PxFilterData& filterData);


	PxQueryHitType::Enum WheelSceneQueryPreFilterBlocking
	(PxFilterData filterData0, PxFilterData filterData1,
		const void* constantBlock, PxU32 constantBlockSize,
		PxHitFlags& queryFlags);

	PxQueryHitType::Enum WheelSceneQueryPostFilterBlocking
	(PxFilterData queryFilterData, PxFilterData objectFilterData,
		const void* constantBlock, PxU32 constantBlockSize,
		const PxQueryHit& hit);

	PxQueryHitType::Enum WheelSceneQueryPreFilterNonBlocking
	(PxFilterData filterData0, PxFilterData filterData1,
		const void* constantBlock, PxU32 constantBlockSize,
		PxHitFlags& queryFlags);

	PxQueryHitType::Enum WheelSceneQueryPostFilterNonBlocking
	(PxFilterData queryFilterData, PxFilterData objectFilterData,
		const void* constantBlock, PxU32 constantBlockSize,
		const PxQueryHit& hit);


	//Data structure for quick setup of scene queries for suspension queries.
	class VehicleSceneQueryData
	{
	public:
		VehicleSceneQueryData();
		~VehicleSceneQueryData();

		//Allocate scene query data for up to maxNumVehicles and up to maxNumWheelsPerVehicle with numVehiclesInBatch per batch query.
		static VehicleSceneQueryData* allocate
		(const PxU32 maxNumVehicles, const PxU32 maxNumWheelsPerVehicle, const PxU32 maxNumHitPointsPerWheel, const PxU32 numVehiclesInBatch,
			PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader);

		//Free allocated buffers.
		void free(PxAllocatorCallback& allocator);

		//Create a PxBatchQuery instance that will be used for a single specified batch.
		static PxBatchQuery* setUpBatchedSceneQuery(const PxU32 batchId, const VehicleSceneQueryData& vehicleSceneQueryData, PxScene* scene);

		//Return an array of scene query results for a single specified batch.
		PxRaycastQueryResult* getRaycastQueryResultBuffer(const PxU32 batchId); 

		//Return an array of scene query results for a single specified batch.
		PxSweepQueryResult* getSweepQueryResultBuffer(const PxU32 batchId); 

		//Get the number of scene query results that have been allocated for a single batch.
		PxU32 getQueryResultBufferSize() const; 

	public:

		//Number of queries per batch
		PxU32 mNumQueriesPerBatch;

		//Number of hit results per query
		PxU32 mNumHitResultsPerQuery;

		//One result for each wheel.
		PxRaycastQueryResult* mRaycastResults;
		PxSweepQueryResult* mSweepResults;

		//One hit for each wheel.
		PxRaycastHit* mRaycastHitBuffer;
		PxSweepHit* mSweepHitBuffer;

		//Filter shader used to filter drivable and non-drivable surfaces
		PxBatchQueryPreFilterShader mPreFilterShader;

		//Filter shader used to reject hit shapes that initially overlap sweeps.
		PxBatchQueryPostFilterShader mPostFilterShader;

	};

} // namespace snippetvehicle

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
		virtual void DoSpawn() override;

		void InitializeSceneBatchQuery(const snippetvehicle::VehicleSceneQueryData& vehicleSceneQueryData);

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
