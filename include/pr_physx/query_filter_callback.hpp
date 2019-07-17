#ifndef __PR_PX_QUERY_FILTER_CALLBACK_HPP__
#define __PR_PX_QUERY_FILTER_CALLBACK_HPP__

#include "pr_physx/common.hpp"

namespace pragma::physics
{
	class ICollisionObject;
	class PhysXQueryFilterCallback
		: public physx::PxQueryFilterCallback
	{
	public:
		PhysXQueryFilterCallback(ICollisionObject &collisionObject);
		/**
		\brief This filter callback is executed before the exact intersection test if PxQueryFlag::ePREFILTER flag was set.

		\param[in] filterData custom filter data specified as the query's filterData.data parameter.
		\param[in] shape A shape that has not yet passed the exact intersection test.
		\param[in] actor The shape's actor.
		\param[in,out] queryFlags scene query flags from the query's function call (only flags from PxHitFlag::eMODIFIABLE_FLAGS bitmask can be modified)
		\return the updated type for this hit  (see #PxQueryHitType)
		*/
		virtual physx::PxQueryHitType::Enum preFilter(
			const physx::PxFilterData& filterData, const physx::PxShape* shape, const physx::PxRigidActor* actor, physx::PxHitFlags& queryFlags) override;

		/**
		\brief This filter callback is executed if the exact intersection test returned true and PxQueryFlag::ePOSTFILTER flag was set.

		\param[in] filterData custom filter data of the query
		\param[in] hit Scene query hit information. faceIndex member is not valid for overlap queries. For sweep and raycast queries the hit information can be cast to #PxSweepHit and #PxRaycastHit respectively.
		\return the updated hit type for this hit  (see #PxQueryHitType)
		*/
		virtual physx::PxQueryHitType::Enum postFilter(const physx::PxFilterData& filterData, const physx::PxQueryHit& hit) override;
	private:
		ICollisionObject &m_collisionObject;
	};

	enum
	{
		DRIVABLE_SURFACE = 64,
		UNDRIVABLE_SURFACE = 32
	};

	physx::PxQueryHitType::Enum BatchQueryPreFilterBlocking(
		physx::PxFilterData filterData0,physx::PxFilterData filterData1,
		const void *constantBlock,physx::PxU32 constantBlockSize,physx::PxHitFlags &queryFlags
	);
};

#endif
