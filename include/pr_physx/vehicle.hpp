#ifndef __PR_PX_VEHICLE_HPP__
#define __PR_PX_VEHICLE_HPP__

#include "pr_physx/common.hpp"
#include <pragma/physics/vehicle.hpp>

// TODO
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
		VehicleSceneQueryData()=default;
		~VehicleSceneQueryData()=default;

		//Allocate scene query data for up to maxNumVehicles and up to maxNumWheelsPerVehicle with numVehiclesInBatch per batch query.
		static VehicleSceneQueryData allocate
		(const PxU32 maxNumVehicles, const PxU32 maxNumWheelsPerVehicle, const PxU32 maxNumHitPointsPerWheel, const PxU32 numVehiclesInBatch,
			PxBatchQueryPreFilterShader preFilterShader, PxBatchQueryPostFilterShader postFilterShader);

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
		PxU32 mNumQueriesPerBatch = 0;

		//Number of hit results per query
		PxU32 mNumHitResultsPerQuery = 0;

		//One result for each wheel.
		std::vector<PxRaycastQueryResult> mRaycastResults = {};
		std::vector<PxSweepQueryResult> mSweepResults = {};

		//One hit for each wheel.
		std::vector<PxRaycastHit> mRaycastHitBuffer = {};
		std::vector<PxSweepHit> mSweepHitBuffer = {};

		//Filter shader used to filter drivable and non-drivable surfaces
		PxBatchQueryPreFilterShader mPreFilterShader = nullptr;

		//Filter shader used to reject hit shapes that initially overlap sweeps.
		PxBatchQueryPostFilterShader mPostFilterShader = nullptr;

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
		enum class StateFlags : uint32_t
		{
			None = 0u,
			InAir = 1u,
			UseDigitalInputs = InAir<<1u
		};

		friend PhysXEnvironment;
		friend IEnvironment;
		static PhysXVehicle &GetVehicle(IVehicle &v);
		static const PhysXVehicle &GetVehicle(const IVehicle &v);
		physx::PxVehicleDrive &GetInternalObject() const;
		PhysXEnvironment &GetPxEnv() const;

		virtual void SetUseDigitalInputs(bool bUseDigitalInputs) override;

		virtual void SetBrakeFactor(float f) override;
		virtual void SetHandbrakeFactor(float f) override;
		virtual void SetAccelerationFactor(float f) override;
		virtual void SetTurnFactor(float f) override;

		virtual void SetGear(Gear gear) override;
		virtual void SetGearDown() override;
		virtual void SetGearUp() override;
		virtual void SetGearSwitchTime(float time) override;
		virtual void ChangeToGear(Gear gear) override;
		virtual void SetUseAutoGears(bool useAutoGears) override;

		virtual bool ShouldUseAutoGears() const override;
		virtual Gear GetCurrentGear() const override;
		virtual float GetEngineRotationSpeed() const override;

		virtual void SetRestState() override;

		virtual void ResetControls() override;

		virtual void SetWheelRotationAngle(WheelIndex wheel,umath::Radian angle) override;
		virtual void SetWheelRotationSpeed(WheelIndex wheel,umath::Radian speed) override;

		virtual bool IsInAir() const override;

		virtual uint32_t GetWheelCount() const override;
		virtual float GetForwardSpeed() const override;
		virtual float GetSidewaysSpeed() const override;
	protected:
		virtual bool ShouldUseDigitalInputs() const override;
	private:
		PhysXVehicle(
			IEnvironment &env,PhysXUniquePtr<physx::PxVehicleDrive> vhc,const util::TSharedHandle<ICollisionObject> &collisionObject,
			PhysXUniquePtr<physx::PxBatchQuery> raycastBatchQuery
		);
		static constexpr bool AnalogInputToDigital(float fAnalog);
		static constexpr physx::PxU32 ToPhysXGear(Gear gear);
		static constexpr Gear FromPhysXGear(physx::PxU32 gear);
		virtual void Initialize() override;
		virtual void RemoveWorldObject() override;
		virtual void DoAddWorldObject() override;
		virtual void DoSpawn() override;

		void InitializeSceneBatchQuery(const snippetvehicle::VehicleSceneQueryData& vehicleSceneQueryData);

		void Simulate(float dt);
		PhysXUniquePtr<physx::PxVehicleDrive> m_vehicle = px_null_ptr<physx::PxVehicleDrive>();
		PhysXUniquePtr<physx::PxBatchQuery> m_raycastBatchQuery = px_null_ptr<physx::PxBatchQuery>();
		StateFlags m_stateFlags = StateFlags::None;
		physx::PxVehicleDrive4WRawInputData m_inputData = {};

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
REGISTER_BASIC_BITWISE_OPERATORS(pragma::physics::PhysXVehicle::StateFlags)

constexpr bool pragma::physics::PhysXVehicle::AnalogInputToDigital(float fAnalog)
{
	return fAnalog > 0.f ? true : false;
}
constexpr physx::PxU32 pragma::physics::PhysXVehicle::ToPhysXGear(Gear gear)
{
	switch(gear)
	{
	case Gear::Reverse:
		return physx::PxVehicleGearsData::eREVERSE;
	case Gear::Neutral:
		return physx::PxVehicleGearsData::eNEUTRAL;
	case Gear::First:
		return physx::PxVehicleGearsData::eFIRST;
	case Gear::Second:
		return physx::PxVehicleGearsData::eSECOND;
	case Gear::Third:
		return physx::PxVehicleGearsData::eTHIRD;
	case Gear::Fourth:
		return physx::PxVehicleGearsData::eFOURTH;
	case Gear::Fifth:
		return physx::PxVehicleGearsData::eFIFTH;
	case Gear::Sixth:
		return physx::PxVehicleGearsData::eSIXTH;
	case Gear::Seventh:
		return physx::PxVehicleGearsData::eSEVENTH;
	case Gear::Eighth:
		return physx::PxVehicleGearsData::eEIGHTH;
	case Gear::Ninth:
		return physx::PxVehicleGearsData::eNINTH;
	case Gear::Tenth:
		return physx::PxVehicleGearsData::eTENTH;
	case Gear::Eleventh:
		return physx::PxVehicleGearsData::eELEVENTH;
	case Gear::Twelfth:
		return physx::PxVehicleGearsData::eTWELFTH;
	case Gear::Thirteenth:
		return physx::PxVehicleGearsData::eTHIRTEENTH;
	case Gear::Fourteenth:
		return physx::PxVehicleGearsData::eFOURTEENTH;
	case Gear::Fifteenth:
		return physx::PxVehicleGearsData::eFIFTEENTH;
	case Gear::Sixteenth:
		return physx::PxVehicleGearsData::eSIXTEENTH;
	case Gear::Seventeenth:
		return physx::PxVehicleGearsData::eSEVENTEENTH;
	case Gear::Eighteenth:
		return physx::PxVehicleGearsData::eEIGHTEENTH;
	case Gear::Nineteenth:
		return physx::PxVehicleGearsData::eNINETEENTH;
	case Gear::Twentieth:
		return physx::PxVehicleGearsData::eTWENTIETH;
	case Gear::Twentyfirst:
		return physx::PxVehicleGearsData::eTWENTYFIRST;
	case Gear::Twentysecond:
		return physx::PxVehicleGearsData::eTWENTYSECOND;
	case Gear::Twentythird:
		return physx::PxVehicleGearsData::eTWENTYTHIRD;
	case Gear::Twentyfourth:
		return physx::PxVehicleGearsData::eTWENTYFOURTH;
	case Gear::Twentyfifth:
		return physx::PxVehicleGearsData::eTWENTYFIFTH;
	case Gear::Twentysixth:
		return physx::PxVehicleGearsData::eTWENTYSIXTH;
	case Gear::Twentyseventh:
		return physx::PxVehicleGearsData::eTWENTYSEVENTH;
	case Gear::Twentyeighth:
		return physx::PxVehicleGearsData::eTWENTYEIGHTH;
	case Gear::Twentyninth:
		return physx::PxVehicleGearsData::eTWENTYNINTH;
	case Gear::Thirtieth:
	default:
		return physx::PxVehicleGearsData::eTHIRTIETH;
	};
}
constexpr pragma::physics::PhysXVehicle::Gear pragma::physics::PhysXVehicle::FromPhysXGear(physx::PxU32 gear)
{
	switch(gear)
	{
	case physx::PxVehicleGearsData::eREVERSE:
		return Gear::Reverse;
	case physx::PxVehicleGearsData::eNEUTRAL:
		return Gear::Neutral;
	case physx::PxVehicleGearsData::eFIRST:
		return Gear::First;
	case physx::PxVehicleGearsData::eSECOND:
		return Gear::Second;
	case physx::PxVehicleGearsData::eTHIRD:
		return Gear::Third;
	case physx::PxVehicleGearsData::eFOURTH:
		return Gear::Fourth;
	case physx::PxVehicleGearsData::eFIFTH:
		return Gear::Fifth;
	case physx::PxVehicleGearsData::eSIXTH:
		return Gear::Sixth;
	case physx::PxVehicleGearsData::eSEVENTH:
		return Gear::Seventh;
	case physx::PxVehicleGearsData::eEIGHTH:
		return Gear::Eighth;
	case physx::PxVehicleGearsData::eNINTH:
		return Gear::Ninth;
	case physx::PxVehicleGearsData::eTENTH:
		return Gear::Tenth;
	case physx::PxVehicleGearsData::eELEVENTH:
		return Gear::Eleventh;
	case physx::PxVehicleGearsData::eTWELFTH:
		return Gear::Twelfth;
	case physx::PxVehicleGearsData::eTHIRTEENTH:
		return Gear::Thirteenth;
	case physx::PxVehicleGearsData::eFOURTEENTH:
		return Gear::Fourteenth;
	case physx::PxVehicleGearsData::eFIFTEENTH:
		return Gear::Fifteenth;
	case physx::PxVehicleGearsData::eSIXTEENTH:
		return Gear::Sixteenth;
	case physx::PxVehicleGearsData::eSEVENTEENTH:
		return Gear::Seventeenth;
	case physx::PxVehicleGearsData::eEIGHTEENTH:
		return Gear::Eighteenth;
	case physx::PxVehicleGearsData::eNINETEENTH:
		return Gear::Nineteenth;
	case physx::PxVehicleGearsData::eTWENTIETH:
		return Gear::Twentieth;
	case physx::PxVehicleGearsData::eTWENTYFIRST:
		return Gear::Twentyfirst;
	case physx::PxVehicleGearsData::eTWENTYSECOND:
		return Gear::Twentysecond;
	case physx::PxVehicleGearsData::eTWENTYTHIRD:
		return Gear::Twentythird;
	case physx::PxVehicleGearsData::eTWENTYFOURTH:
		return Gear::Twentyfourth;
	case physx::PxVehicleGearsData::eTWENTYFIFTH:
		return Gear::Twentyfifth;
	case physx::PxVehicleGearsData::eTWENTYSIXTH:
		return Gear::Twentysixth;
	case physx::PxVehicleGearsData::eTWENTYSEVENTH:
		return Gear::Twentyseventh;
	case physx::PxVehicleGearsData::eTWENTYEIGHTH:
		return Gear::Twentyeighth;
	case physx::PxVehicleGearsData::eTWENTYNINTH:
		return Gear::Twentyninth;
	case physx::PxVehicleGearsData::eTHIRTIETH:
	default:
		return Gear::Thirtieth;
	};
}

#endif
