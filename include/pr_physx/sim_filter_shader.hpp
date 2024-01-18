//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

// Note is is based on the physx::PxDefaultSimulationFilterShader
// example shader from the PhysX SDK

#ifndef __PR_PX_SIM_FILTER_SHADER_HPP__
#define __PR_PX_SIM_FILTER_SHADER_HPP__

#include "pr_physx/common.hpp"
#include <PxPhysXConfig.h>
#include <PxFiltering.h>

class PxActor;

namespace pragma::physics {
	class PhysXGroupsMask {
	  public:
		PX_INLINE PhysXGroupsMask() : bits0(0), bits1(0), bits2(0), bits3(0) {}
		PX_INLINE ~PhysXGroupsMask() {}

		physx::PxU16 bits0, bits1, bits2, bits3;
	};

	/**
	\brief Collision filtering operations.

	@see PxGroupsMask
	*/
	struct PhysXFilterOp {
		enum Enum { PX_FILTEROP_AND, PX_FILTEROP_OR, PX_FILTEROP_XOR, PX_FILTEROP_NAND, PX_FILTEROP_NOR, PX_FILTEROP_NXOR, PX_FILTEROP_SWAP_AND };
	};

	/**
	\brief Implementation of a simple filter shader that emulates PhysX 2.8.x filtering

	This shader provides the following logic:
	\li If one of the two filter objects is a trigger, the pair is acccepted and #PxPairFlag::eTRIGGER_DEFAULT will be used for trigger reports
	\li Else, if the filter mask logic (see further below) discards the pair it will be suppressed (#PxFilterFlag::eSUPPRESS)
	\li Else, the pair gets accepted and collision response gets enabled (#PxPairFlag::eCONTACT_DEFAULT)

	Filter mask logic:
	Given the two #PxFilterData structures fd0 and fd1 of two collision objects, the pair passes the filter if the following
	conditions are met:

		1) Collision groups of the pair are enabled
		2) Collision filtering equation is satisfied

	@see PxSimulationFilterShader
	*/

	physx::PxFilterFlags PhysXSimulationFilterShader(physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0, physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1, physx::PxPairFlags &pairFlags, const void *constantBlock,
	  physx::PxU32 constantBlockSize);

	/**
		\brief Determines if collision detection is performed between a pair of groups

		\note Collision group is an integer between 0 and 31.

		\param[in] group1 First Group
		\param[in] group2 Second Group

		\return True if the groups could collide

		@see PxSetGroupCollisionFlag
	*/
	//bool PhysXGetGroupCollisionFlag(const physx::PxU16 group1, const physx::PxU16 group2);

	/**
		\brief Specifies if collision should be performed by a pair of groups

		\note Collision group is an integer between 0 and 31.

		\param[in] group1 First Group
		\param[in] group2 Second Group
		\param[in] enable True to enable collision between the groups

		@see PxGetGroupCollisionFlag
	*/
	//void PhysXSetGroupCollisionFlag(const physx::PxU16 group1, const physx::PxU16 group2, const bool enable);

	/**
		\brief Retrieves the value set with PxSetGroup()

		\note Collision group is an integer between 0 and 31.

		\param[in] actor The actor

		\return The collision group this actor belongs to

		@see PxSetGroup
	*/
	physx::PxU16 PhysXGetGroup(const physx::PxActor &actor);

	/**
		\brief Sets which collision group this actor is part of

		\note Collision group is an integer between 0 and 31.

		\param[in] actor The actor
		\param[in] collisionGroup Collision group this actor belongs to

		@see PxGetGroup
	*/
	void PhysXSetGroup(physx::PxActor &actor, const physx::PxU16 collisionGroup);

	/**
	\brief Retrieves filtering operation. See comments for PxGroupsMask

	\param[out] op0 First filter operator.
	\param[out] op1 Second filter operator.
	\param[out] op2 Third filter operator.

	@see PxSetFilterOps PxSetFilterBool PxSetFilterConstants
	*/
	void PhysXGetFilterOps(PhysXFilterOp::Enum &op0, PhysXFilterOp::Enum &op1, PhysXFilterOp::Enum &op2);

	/**
	\brief Setups filtering operations. See comments for PxGroupsMask

	\param[in] op0 Filter op 0.
	\param[in] op1 Filter op 1.
	\param[in] op2 Filter op 2.

	@see PxSetFilterBool PxSetFilterConstants
	*/
	void PhysXSetFilterOps(const PhysXFilterOp::Enum &op0, const PhysXFilterOp::Enum &op1, const PhysXFilterOp::Enum &op2);

	/**
	\brief Retrieves filtering's boolean value. See comments for PxGroupsMask

	\return flag Boolean value for filter.

	@see PxSetFilterBool PxSetFilterConstants
	*/
	bool PhysXGetFilterBool();

	/**
	\brief Setups filtering's boolean value. See comments for PxGroupsMask

	\param[in] enable Boolean value for filter.

	@see PxSetFilterOps PxSsetFilterConstants
	*/
	void PhysXSetFilterBool(const bool enable);

	/**
	\brief Gets filtering constant K0 and K1. See comments for PxGroupsMask

	\param[out] c0 the filtering constants, as a mask. See #PxGroupsMask.
	\param[out] c1 the filtering constants, as a mask. See #PxGroupsMask.

	@see PxSetFilterOps PxSetFilterBool PxSetFilterConstants
	*/
	void PhysXGetFilterConstants(PhysXGroupsMask &c0, PhysXGroupsMask &c1);

	/**
	\brief Setups filtering's K0 and K1 value. See comments for PxGroupsMask

	\param[in] c0 The new group mask. See #PxGroupsMask.
	\param[in] c1 The new group mask. See #PxGroupsMask.

	@see PxSetFilterOps PxSetFilterBool PxGetFilterConstants
	*/
	void PhysXSetFilterConstants(const PhysXGroupsMask &c0, const PhysXGroupsMask &c1);

	/**
	\brief Gets 64-bit mask used for collision filtering. See comments for PxGroupsMask

	\param[in] actor The actor

	\return The group mask for the actor.

	@see PxSetGroupsMask()
	*/
	PhysXGroupsMask PhysXGetGroupsMask(const physx::PxActor &actor);

	/**
	\brief Sets 64-bit mask used for collision filtering. See comments for PxGroupsMask

	\param[in] actor The actor
	\param[in] mask The group mask to set for the actor.

	@see PxGetGroupsMask()
	*/
	void PhysXSetGroupsMask(physx::PxActor &actor, const PhysXGroupsMask &mask);
};

#endif
