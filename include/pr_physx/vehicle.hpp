#ifndef __PR_PX_VEHICLE_HPP__
#define __PR_PX_VEHICLE_HPP__

#include "pr_physx/common.hpp"
#include <pragma/physics/vehicle.hpp>
#include <vector>

namespace pragma::physics
{
	class PhysXEnvironment;
	class VehicleSceneQueryData
	{
	public:
		VehicleSceneQueryData(uint32_t numQueriesInBatch);
		~VehicleSceneQueryData()=default;

		//Allocate scene query data for up to maxNumVehicles and up to maxNumWheelsPerVehicle with numVehiclesInBatch per batch query.
		static std::unique_ptr<VehicleSceneQueryData> Create(pragma::physics::PhysXEnvironment &env,uint32_t numWheels,physx::PxBatchQueryPreFilterShader preFilterShader,physx::PxBatchQueryPostFilterShader postFilterShader);

		const std::vector<physx::PxRaycastQueryResult> &GetRaycastResults() const;
		std::vector<physx::PxRaycastQueryResult> &GetRaycastResults();
		const std::vector<physx::PxSweepQueryResult> &GetSweepResults() const;
		std::vector<physx::PxSweepQueryResult> &GetSweepResults();
		const std::vector<physx::PxRaycastHit> &GetRaycastHits() const;
		std::vector<physx::PxRaycastHit> &GetRaycastHits();
		const std::vector<physx::PxSweepHit> &GetSweepHits() const;
		std::vector<physx::PxSweepHit> &GetSweepHits();

		physx::PxBatchQuery &GetBatchQuery();

		const physx::PxBatchQueryPreFilterShader &GetPreFilterShader() const;
		const physx::PxBatchQueryPostFilterShader &GetPostFilterShader() const;

		physx::PxU32 GetNumQueriesPerBatch() const;
		physx::PxU32 GetNumHitResultsPerQuery() const;
	private:

		//Number of queries per batch
		physx::PxU32 m_numQueriesPerBatch = 0;

		//Number of hit results per query
		physx::PxU32 m_numHitResultsPerQuery = 0;

		//One result for each wheel.
		std::vector<physx::PxRaycastQueryResult> m_raycastResults = {};
		std::vector<physx::PxSweepQueryResult> m_sweepResults = {};

		//One hit for each wheel.
		std::vector<physx::PxRaycastHit> m_raycastHitBuffer = {};
		std::vector<physx::PxSweepHit> m_sweepHitBuffer = {};

		//Filter shader used to filter drivable and non-drivable surfaces
		physx::PxBatchQueryPreFilterShader m_preFilterShader = nullptr;

		//Filter shader used to reject hit shapes that initially overlap sweeps.
		physx::PxBatchQueryPostFilterShader m_postFilterShader = nullptr;

		physx::PxBatchQueryDesc m_batchQueryDesc;
		pragma::physics::PhysXUniquePtr<physx::PxBatchQuery> m_batchQuery = pragma::physics::px_null_ptr<physx::PxBatchQuery>();
	};

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
			std::unique_ptr<pragma::physics::VehicleSceneQueryData> vhcSceneQueryData,const physx::PxFixedSizeLookupTable<8> &steerVsForwardSpeedTable
		);
		static constexpr bool AnalogInputToDigital(float fAnalog);
		static constexpr physx::PxU32 ToPhysXGear(Gear gear);
		static constexpr Gear FromPhysXGear(physx::PxU32 gear);
		virtual void Initialize() override;
		virtual void RemoveWorldObject() override;
		virtual void DoAddWorldObject() override;
		virtual void DoSpawn() override;

		void Simulate(float dt);
		PhysXUniquePtr<physx::PxVehicleDrive> m_vehicle = px_null_ptr<physx::PxVehicleDrive>();
		StateFlags m_stateFlags = StateFlags::None;
		physx::PxVehicleDrive4WRawInputData m_inputData = {};

		std::vector<physx::PxWheelQueryResult> m_wheelQueryResults;
		std::unique_ptr<pragma::physics::VehicleSceneQueryData> m_vehicleSceneQuery = nullptr;
		physx::PxFixedSizeLookupTable<8> m_steerVsForwardSpeedTable;
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
