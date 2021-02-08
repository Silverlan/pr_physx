/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef __PR_PX_RAYCAST_HPP__
#define __PR_PX_RAYCAST_HPP__

#include <PxPhysicsAPI.h>
#include <pragma/physics/raycast_filter.hpp>

enum class RayCastHitType : uint8_t;
namespace pragma::physics
{
	class IRayCastFilterCallback;
	class PhysXEnvironment;
	class RayCastFilterCallback
		: public physx::PxQueryFilterCallback
	{
	public:
		RayCastFilterCallback(const PhysXEnvironment &env,IRayCastFilterCallback &rayCastFilterCallback,bool invertResult);
		/**
		\brief This filter callback is executed before the exact intersection test if PxQueryFlag::ePREFILTER flag was set.

		\param[in] filterData custom filter data specified as the query's filterData.data parameter.
		\param[in] shape A shape that has not yet passed the exact intersection test.
		\param[in] actor The shape's actor.
		\param[in,out] queryFlags scene query flags from the query's function call (only flags from PxHitFlag::eMODIFIABLE_FLAGS bitmask can be modified)
		\return the updated type for this hit  (see #PxQueryHitType)
		*/
		virtual physx::PxQueryHitType::Enum preFilter(const physx::PxFilterData& filterData, const physx::PxShape* shape, const physx::PxRigidActor* actor, physx::PxHitFlags& queryFlags) override;

		/**
		\brief This filter callback is executed if the exact intersection test returned true and PxQueryFlag::ePOSTFILTER flag was set.

		\param[in] filterData custom filter data of the query
		\param[in] hit Scene query hit information. faceIndex member is not valid for overlap queries. For sweep and raycast queries the hit information can be cast to #PxSweepHit and #PxRaycastHit respectively.
		\return the updated hit type for this hit  (see #PxQueryHitType)
		*/
		virtual physx::PxQueryHitType::Enum postFilter(const physx::PxFilterData& filterData, const physx::PxQueryHit& hit) override;
	private:
		physx::PxQueryHitType::Enum Filter(const physx::PxShape *shape,const physx::PxRigidActor *actor,RayCastHitType(IRayCastFilterCallback::*filter)(IShape&,IRigidBody&) const);

		const PhysXEnvironment &m_env;
		IRayCastFilterCallback &m_rayCastFilterCallback;
		bool m_bInvertResult = false;
	};
};

#endif
