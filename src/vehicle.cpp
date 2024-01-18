/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "pr_physx/vehicle.hpp"
#include "pr_physx/environment.hpp"
#include "pr_physx/collision_object.hpp"
#include "pr_physx/shape.hpp"
#include "pr_physx/query_filter_callback.hpp"
#include <vehicle/PxVehicleUtil.h>

pragma::physics::PhysXVehicle &pragma::physics::PhysXVehicle::GetVehicle(IVehicle &c) { return *static_cast<PhysXVehicle *>(c.GetUserData()); }
const pragma::physics::PhysXVehicle &pragma::physics::PhysXVehicle::GetVehicle(const pragma::physics::IVehicle &v) { return GetVehicle(const_cast<pragma::physics::IVehicle &>(v)); }
pragma::physics::PhysXVehicle::PhysXVehicle(IEnvironment &env, PhysXUniquePtr<physx::PxVehicleDrive> vhc, const util::TSharedHandle<ICollisionObject> &collisionObject,
#if ENABLE_PX_VEHICLES
  std::unique_ptr<pragma::physics::VehicleSceneQueryData> vhcSceneQueryData,
#endif
  const physx::PxFixedSizeLookupTable<8> &steerVsForwardSpeedTable, const VehicleCreateInfo &createInfo)
    : IVehicle {env, collisionObject}, m_vehicle {std::move(vhc)},
#if ENABLE_PX_VEHICLES
      m_vehicleSceneQuery {std::move(vhcSceneQueryData)},
#endif
      m_steerVsForwardSpeedTable {steerVsForwardSpeedTable}, m_createInfo {createInfo}
{
	SetUserData(this);
}
physx::PxVehicleDrive &pragma::physics::PhysXVehicle::GetInternalObject() const { return *m_vehicle; }

void pragma::physics::PhysXVehicle::Simulate(float dt)
{
	if(IsSpawned() == false)
		return;
	auto *vhc4w = static_cast<physx::PxVehicleDrive4W *>(m_vehicle.get());
	auto *actor = vhc4w->getRigidDynamicActor();
	if(actor == nullptr)
		return;
	// Source: PhysX vehicle demo samples
	static constexpr physx::PxVehicleKeySmoothingData keySmoothingData = {{
	                                                                        6.0f, //rise rate eANALOG_INPUT_ACCEL
	                                                                        6.0f, //rise rate eANALOG_INPUT_BRAKE
	                                                                        6.0f, //rise rate eANALOG_INPUT_HANDBRAKE
	                                                                        2.5f, //rise rate eANALOG_INPUT_STEER_LEFT
	                                                                        2.5f, //rise rate eANALOG_INPUT_STEER_RIGHT
	                                                                      },
	  {
	    10.0f, //fall rate eANALOG_INPUT_ACCEL
	    10.0f, //fall rate eANALOG_INPUT_BRAKE
	    10.0f, //fall rate eANALOG_INPUT_HANDBRAKE
	    5.0f,  //fall rate eANALOG_INPUT_STEER_LEFT
	    5.0f   //fall rate eANALOG_INPUT_STEER_RIGHT
	  }};

	static constexpr physx::PxVehiclePadSmoothingData padSmoothingData = {{
	                                                                        6.0f, //rise rate eANALOG_INPUT_ACCEL
	                                                                        6.0f, //rise rate eANALOG_INPUT_BRAKE
	                                                                        6.0f, //rise rate eANALOG_INPUT_HANDBRAKE
	                                                                        2.5f, //rise rate eANALOG_INPUT_STEER_LEFT
	                                                                        2.5f, //rise rate eANALOG_INPUT_STEER_RIGHT
	                                                                      },
	  {
	    10.0f, //fall rate eANALOG_INPUT_ACCEL
	    10.0f, //fall rate eANALOG_INPUT_BRAKE
	    10.0f, //fall rate eANALOG_INPUT_HANDBRAKE
	    5.0f,  //fall rate eANALOG_INPUT_STEER_LEFT
	    5.0f   //fall rate eANALOG_INPUT_STEER_RIGHT
	  }};

	if(ShouldUseDigitalInputs())
		physx::PxVehicleDrive4WSmoothDigitalRawInputsAndSetAnalogInputs(keySmoothingData, m_steerVsForwardSpeedTable, m_inputData, dt, IsInAir(), *vhc4w);
	else
		physx::PxVehicleDrive4WSmoothAnalogRawInputsAndSetAnalogInputs(padSmoothingData, m_steerVsForwardSpeedTable, m_inputData, dt, IsInAir(), *vhc4w);

	std::array<physx::PxVehicleWheels *, 1> vehicles = {vhc4w};
#if ENABLE_PX_VEHICLES
	auto &raycastResults = m_vehicleSceneQuery->GetRaycastResults();
	physx::PxVehicleSuspensionRaycasts(&m_vehicleSceneQuery->GetBatchQuery(), vehicles.size(), vehicles.data(), raycastResults.size(), raycastResults.data());
#endif

	physx::PxVec3 gravity {};
	if(actor)
		gravity = GetPxEnv().ToPhysXVector(PhysXEnvironment::GetCollisionObject(*actor)->GetGravity());
	if(gravity.magnitudeSquared() == 0.f)
		gravity = {0.f, -0.0001f, 0.f}; // Zero gravity is not allowed by PhysX
	std::array<physx::PxVehicleWheelQueryResult, 1> vehicleQueryResults = {{m_wheelQueryResults.data(), m_wheelQueryResults.size()}};
	PxVehicleUpdates(dt, gravity, GetPxEnv().GetVehicleSurfaceTireFrictionPairs(), vehicles.size(), vehicles.data(), vehicleQueryResults.data());

	//Work out if the vehicle is in the air.
	umath::set_flag(m_stateFlags, StateFlags::InAir, vhc4w->getRigidDynamicActor()->isSleeping() ? false : PxVehicleIsInAir(vehicleQueryResults.front()));
}
void pragma::physics::PhysXVehicle::Initialize()
{
	IVehicle::Initialize();
	auto numWheels = GetWheelCount();
	m_wheelQueryResults.resize(numWheels);
}
std::optional<umath::Transform> pragma::physics::PhysXVehicle::GetLocalWheelPose(uint32_t wheelIndex) const
{
	auto *colObj = GetCollisionObject();
	if(colObj == nullptr || wheelIndex >= m_createInfo.wheels.size())
		return {};
	auto &wheelCreateInfo = m_createInfo.wheels.at(wheelIndex);
	auto &physXColObj = PhysXCollisionObject::GetCollisionObject(*colObj);
	auto &actorShapes = physXColObj.GetActorShapeCollection().GetActorShapes();
	if(wheelCreateInfo.shapeIndex < 0 || wheelCreateInfo.shapeIndex >= actorShapes.size())
		return {};
	return actorShapes.at(wheelCreateInfo.shapeIndex)->GetLocalPose();
}
uint32_t pragma::physics::PhysXVehicle::GetWheelCount() const { return 4; }
umath::Radian pragma::physics::PhysXVehicle::GetWheelYawAngle(WheelIndex wheel) const
{
	auto pose = GetLocalWheelPose(wheel);
	if(pose.has_value() == false)
		return 0.f;
	return umath::deg_to_rad(EulerAngles {pose->GetRotation()}.y);
}
umath::Radian pragma::physics::PhysXVehicle::GetWheelRollAngle(WheelIndex wheel) const
{
	if(wheel >= m_vehicle->mWheelsDynData.getNbWheelRotationAngle())
		return 0.f;
	return m_vehicle->mWheelsDynData.getWheelRotationAngle(wheel);
}
float pragma::physics::PhysXVehicle::GetSteerFactor() const
{
	if(ShouldUseDigitalInputs()) {
		auto steerFactor = 0.f;
		if(m_inputData.getDigitalSteerLeft())
			steerFactor += 1.f;
		if(m_inputData.getDigitalSteerRight())
			steerFactor -= 1.f;
		return steerFactor;
	}
	return m_inputData.getAnalogSteer();
}
float pragma::physics::PhysXVehicle::GetBrakeFactor() const
{
	if(ShouldUseDigitalInputs())
		return DigitalInputToAnalog(m_inputData.getDigitalBrake());
	return m_inputData.getAnalogBrake();
}
float pragma::physics::PhysXVehicle::GetHandbrakeFactor() const
{
	if(ShouldUseDigitalInputs())
		return DigitalInputToAnalog(m_inputData.getDigitalHandbrake());
	return m_inputData.getAnalogHandbrake();
}
float pragma::physics::PhysXVehicle::GetAccelerationFactor() const
{
	if(ShouldUseDigitalInputs())
		return DigitalInputToAnalog(m_inputData.getDigitalAccel());
	return m_inputData.getAnalogAccel();
}
umath::Radian pragma::physics::PhysXVehicle::GetWheelRotationSpeed(WheelIndex wheel) const { return m_vehicle->mWheelsDynData.getWheelRotationSpeed(wheel); }
float pragma::physics::PhysXVehicle::GetForwardSpeed() const { return GetPxEnv().FromPhysXLength(m_vehicle->computeForwardSpeed()); }
float pragma::physics::PhysXVehicle::GetSidewaysSpeed() const { return GetPxEnv().FromPhysXLength(m_vehicle->computeSidewaysSpeed()); }
void pragma::physics::PhysXVehicle::RemoveWorldObject()
{
	if(m_collisionObject == nullptr)
		return;
	m_collisionObject->RemoveWorldObject();
}
void pragma::physics::PhysXVehicle::DoAddWorldObject() {}
void pragma::physics::PhysXVehicle::DoSpawn()
{
	IVehicle::DoSpawn();
	if(m_collisionObject == nullptr)
		return;
	m_collisionObject->Spawn();
}
pragma::physics::PhysXEnvironment &pragma::physics::PhysXVehicle::GetPxEnv() const { return static_cast<PhysXEnvironment &>(m_physEnv); }

void pragma::physics::PhysXVehicle::SetUseDigitalInputs(bool bUseDigitalInputs) { umath::set_flag(m_stateFlags, StateFlags::UseDigitalInputs, bUseDigitalInputs); }
bool pragma::physics::PhysXVehicle::ShouldUseDigitalInputs() const { return umath::is_flag_set(m_stateFlags, StateFlags::UseDigitalInputs); }

void pragma::physics::PhysXVehicle::SetBrakeFactor(float f)
{
	if(ShouldUseDigitalInputs())
		m_inputData.setDigitalBrake(AnalogInputToDigital(f));
	else
		m_inputData.setAnalogBrake(f);
}
void pragma::physics::PhysXVehicle::SetHandbrakeFactor(float f)
{
	if(ShouldUseDigitalInputs())
		m_inputData.setDigitalHandbrake(AnalogInputToDigital(f));
	else
		m_inputData.setAnalogHandbrake(f);
}
void pragma::physics::PhysXVehicle::SetAccelerationFactor(float f)
{
	auto gear = GetCurrentGear();
	if(f < 0.f) {
		f = -f;
		if(gear != Gear::Reverse)
			SetGear(Gear::Reverse);
	}
	else if(f > 0.f && gear == Gear::Reverse)
		SetGear(Gear::First);
	if(ShouldUseDigitalInputs())
		m_inputData.setDigitalAccel(AnalogInputToDigital(f));
	else
		m_inputData.setAnalogAccel(f);
}
void pragma::physics::PhysXVehicle::SetSteerFactor(float f)
{
	if(ShouldUseDigitalInputs()) {
		m_inputData.setDigitalSteerRight(AnalogInputToDigital(-f));
		m_inputData.setDigitalSteerLeft(AnalogInputToDigital(f));
	}
	else
		m_inputData.setAnalogSteer(f);
}

void pragma::physics::PhysXVehicle::ResetControls()
{
	if(ShouldUseDigitalInputs()) {
		m_inputData.setDigitalAccel(false);
		m_inputData.setDigitalSteerLeft(false);
		m_inputData.setDigitalSteerRight(false);
		m_inputData.setDigitalBrake(false);
		m_inputData.setDigitalHandbrake(false);
	}
	else {
		m_inputData.setAnalogAccel(0.0f);
		m_inputData.setAnalogSteer(0.0f);
		m_inputData.setAnalogBrake(0.0f);
		m_inputData.setAnalogHandbrake(0.0f);
	}
}

void pragma::physics::PhysXVehicle::SetGear(Gear gear) { m_vehicle->mDriveDynData.forceGearChange(ToPhysXGear(gear)); }
void pragma::physics::PhysXVehicle::SetGearDown() { m_inputData.setGearDown(true); }
void pragma::physics::PhysXVehicle::SetGearUp() { m_inputData.setGearUp(true); }
void pragma::physics::PhysXVehicle::SetGearSwitchTime(float time) { m_vehicle->mDriveDynData.setGearSwitchTime(time); }
void pragma::physics::PhysXVehicle::ChangeToGear(Gear gear) { m_vehicle->mDriveDynData.setTargetGear(ToPhysXGear(gear)); }
void pragma::physics::PhysXVehicle::SetUseAutoGears(bool useAutoGears) { m_vehicle->mDriveDynData.setUseAutoGears(useAutoGears); }

bool pragma::physics::PhysXVehicle::ShouldUseAutoGears() const { return m_vehicle->mDriveDynData.getUseAutoGears(); }
pragma::physics::PhysXVehicle::Gear pragma::physics::PhysXVehicle::GetCurrentGear() const { return FromPhysXGear(m_vehicle->mDriveDynData.getCurrentGear()); }
umath::Radian pragma::physics::PhysXVehicle::GetEngineRotationSpeed() const { return m_vehicle->mDriveDynData.getEngineRotationSpeed(); }

void pragma::physics::PhysXVehicle::SetEngineRotationSpeed(umath::Radian speed) const { m_vehicle->mDriveDynData.setEngineRotationSpeed(speed); }

void pragma::physics::PhysXVehicle::SetRestState() { m_vehicle->mDriveDynData.setToRestState(); }

void pragma::physics::PhysXVehicle::SetWheelRotationAngle(WheelIndex wheel, umath::Radian angle) { m_vehicle->mWheelsDynData.setWheelRotationAngle(wheel, angle); }
void pragma::physics::PhysXVehicle::SetWheelRotationSpeed(WheelIndex wheel, umath::Radian speed) { m_vehicle->mWheelsDynData.setWheelRotationSpeed(wheel, speed); }

bool pragma::physics::PhysXVehicle::IsInAir() const { return umath::is_flag_set(m_stateFlags, StateFlags::InAir); }
