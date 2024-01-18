/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef __SIM_EVENT_CALLBACK_HPP__
#define __SIM_EVENT_CALLBACK_HPP__

#include "common.hpp"

namespace pragma::physics {
	class PhysXSimulationEventCallback : public physx::PxSimulationEventCallback {
	  public:
		/**
		\brief This is called when a breakable constraint breaks.

		\note The user should not release the constraint shader inside this call!

		\note No event will get reported if the constraint breaks but gets deleted while the time step is still being simulated.

		\param[in] constraints - The constraints which have been broken.
		\param[in] count       - The number of constraints

		@see PxConstraint PxConstraintDesc.linearBreakForce PxConstraintDesc.angularBreakForce
		*/
		virtual void onConstraintBreak(physx::PxConstraintInfo *constraints, physx::PxU32 count) override;

		/**
		\brief This is called with the actors which have just been woken up.

		\note Only supported by rigid bodies yet.
		\note Only called on actors for which the PxActorFlag eSEND_SLEEP_NOTIFIES has been set.
		\note Only the latest sleep state transition happening between fetchResults() of the previous frame and fetchResults() of the current frame
		will get reported. For example, let us assume actor A is awake, then A->putToSleep() gets called, then later A->wakeUp() gets called.
		At the next simulate/fetchResults() step only an onWake() event will get triggered because that was the last transition.
		\note If an actor gets newly added to a scene with properties such that it is awake and the sleep state does not get changed by 
		the user or simulation, then an onWake() event will get sent at the next simulate/fetchResults() step.

		\param[in] actors - The actors which just woke up.
		\param[in] count  - The number of actors

		@see PxScene.setSimulationEventCallback() PxSceneDesc.simulationEventCallback PxActorFlag PxActor.setActorFlag()
		*/
		virtual void onWake(physx::PxActor **actors, physx::PxU32 count) override;

		/**
		\brief This is called with the actors which have just been put to sleep.

		\note Only supported by rigid bodies yet.
		\note Only called on actors for which the PxActorFlag eSEND_SLEEP_NOTIFIES has been set.
		\note Only the latest sleep state transition happening between fetchResults() of the previous frame and fetchResults() of the current frame
		will get reported. For example, let us assume actor A is asleep, then A->wakeUp() gets called, then later A->putToSleep() gets called.
		At the next simulate/fetchResults() step only an onSleep() event will get triggered because that was the last transition (assuming the simulation
		does not wake the actor up).
		\note If an actor gets newly added to a scene with properties such that it is asleep and the sleep state does not get changed by 
		the user or simulation, then an onSleep() event will get sent at the next simulate/fetchResults() step.

		\param[in] actors - The actors which have just been put to sleep.
		\param[in] count  - The number of actors

		@see PxScene.setSimulationEventCallback() PxSceneDesc.simulationEventCallback PxActorFlag PxActor.setActorFlag()
		*/
		virtual void onSleep(physx::PxActor **actors, physx::PxU32 count) override;

		/**
		\brief This is called when certain contact events occur.

		The method will be called for a pair of actors if one of the colliding shape pairs requested contact notification.
		You request which events are reported using the filter shader/callback mechanism (see #PxSimulationFilterShader,
		#PxSimulationFilterCallback, #PxPairFlag).

		Do not keep references to the passed objects, as they will be 
		invalid after this function returns.

		\param[in] pairHeader Information on the two actors whose shapes triggered a contact report.
		\param[in] pairs The contact pairs of two actors for which contact reports have been requested. See #PxContactPair.
		\param[in] nbPairs The number of provided contact pairs.

		@see PxScene.setSimulationEventCallback() PxSceneDesc.simulationEventCallback PxContactPair PxPairFlag PxSimulationFilterShader PxSimulationFilterCallback
		*/
		virtual void onContact(const physx::PxContactPairHeader &pairHeader, const physx::PxContactPair *pairs, physx::PxU32 nbPairs) override;

		/**
		\brief This is called with the current trigger pair events.

		Shapes which have been marked as triggers using PxShapeFlag::eTRIGGER_SHAPE will send events
		according to the pair flag specification in the filter shader (see #PxPairFlag, #PxSimulationFilterShader).

		\note Trigger shapes will no longer send notification events for interactions with other trigger shapes.

		\param[in] pairs - The trigger pair events.
		\param[in] count - The number of trigger pair events.

		@see PxScene.setSimulationEventCallback() PxSceneDesc.simulationEventCallback PxPairFlag PxSimulationFilterShader PxShapeFlag PxShape.setFlag()
		*/
		virtual void onTrigger(physx::PxTriggerPair *pairs, physx::PxU32 count) override;

		/**
		\brief Provides early access to the new pose of moving rigid bodies.

		When this call occurs, rigid bodies having the #PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW 
		flag set, were moved by the simulation and their new poses can be accessed through the provided buffers.

		\note The provided buffers are valid and can be read until the next call to #PxScene::simulate() or #PxScene::collide().

		\note Buffered user changes to the rigid body pose will not yet be reflected in the provided data. More important,
		the provided data might contain bodies that have been deleted while the simulation was running. It is the user's
		responsibility to detect and avoid dereferencing such bodies.

		\note This callback gets triggered while the simulation is running. If the provided rigid body references are used to
		read properties of the object, then the callback has to guarantee no other thread is writing to the same body at the same
		time.

		\note The code in this callback should be lightweight as it can block the simulation, that is, the
		#PxScene::fetchResults() call.

		\param[in] bodyBuffer The rigid bodies that moved and requested early pose reporting.
		\param[in] poseBuffer The integrated rigid body poses of the bodies listed in bodyBuffer.
		\param[in] count The number of entries in the provided buffers.

		@see PxScene.setSimulationEventCallback() PxSceneDesc.simulationEventCallback PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW
		*/
		virtual void onAdvance(const physx::PxRigidBody *const *bodyBuffer, const physx::PxTransform *poseBuffer, const physx::PxU32 count) override;

		virtual ~PhysXSimulationEventCallback() override;
	};

	class PhysXSimulationFilterCallback : public physx::PxSimulationFilterCallback {
	  public:
		virtual ~PhysXSimulationFilterCallback() override {}
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
		virtual physx::PxFilterFlags pairFound(physx::PxU64 pairID, physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0, const physx::PxActor *a0, const physx::PxShape *s0, physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1,
		  const physx::PxActor *a1, const physx::PxShape *s1, physx::PxPairFlags &pairFlags) override
		{
			return physx::PxFilterFlag::eDEFAULT;
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
		virtual void pairLost(physx::PxU64 pairID, physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0, physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1, bool objectRemoved) override {}

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
		virtual bool statusChange(physx::PxU64 &pairID, physx::PxPairFlags &pairFlags, physx::PxFilterFlags &filterFlags) override { return false; }
	};
};

#endif
