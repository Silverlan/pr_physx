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

#include "pr_physx/sim_filter_shader.hpp"
#include "pr_physx/environment.hpp"

using namespace physx;

#pragma optimize("",off)
namespace pragma::physics
{
//#define GROUP_SIZE	32

	struct PhysXCollisionBitMap
	{
		PX_INLINE PhysXCollisionBitMap() : enable(true) {}

		bool operator()() const { return enable; }
		bool& operator= (const bool &v) { enable = v; return enable; } 

	private:
		bool enable;
	};

	//PhysXCollisionBitMap gCollisionTable[GROUP_SIZE][GROUP_SIZE];

	PhysXFilterOp::Enum gFilterOps[3] = { PhysXFilterOp::PX_FILTEROP_AND, PhysXFilterOp::PX_FILTEROP_AND, PhysXFilterOp::PX_FILTEROP_AND };

	PhysXGroupsMask gFilterConstants[2];

	bool gFilterBool = false;

	static void gAND(PhysXGroupsMask& results, const PhysXGroupsMask& mask0, const PhysXGroupsMask& mask1)
	{
		results.bits0 = PxU16(mask0.bits0 & mask1.bits0);
		results.bits1 = PxU16(mask0.bits1 & mask1.bits1);
		results.bits2 = PxU16(mask0.bits2 & mask1.bits2);
		results.bits3 = PxU16(mask0.bits3 & mask1.bits3);
	}
	static void gOR(PhysXGroupsMask& results, const PhysXGroupsMask& mask0, const PhysXGroupsMask& mask1)
	{
		results.bits0 = PxU16(mask0.bits0 | mask1.bits0);
		results.bits1 = PxU16(mask0.bits1 | mask1.bits1);
		results.bits2 = PxU16(mask0.bits2 | mask1.bits2);
		results.bits3 = PxU16(mask0.bits3 | mask1.bits3);
	}
	static void gXOR(PhysXGroupsMask& results, const PhysXGroupsMask& mask0, const PhysXGroupsMask& mask1)
	{
		results.bits0 = PxU16(mask0.bits0 ^ mask1.bits0);
		results.bits1 = PxU16(mask0.bits1 ^ mask1.bits1);
		results.bits2 = PxU16(mask0.bits2 ^ mask1.bits2);
		results.bits3 = PxU16(mask0.bits3 ^ mask1.bits3);
	}
	static void gNAND(PhysXGroupsMask& results, const PhysXGroupsMask& mask0, const PhysXGroupsMask& mask1)
	{
		results.bits0 = PxU16(~(mask0.bits0 & mask1.bits0));
		results.bits1 = PxU16(~(mask0.bits1 & mask1.bits1));
		results.bits2 = PxU16(~(mask0.bits2 & mask1.bits2));
		results.bits3 = PxU16(~(mask0.bits3 & mask1.bits3));
	}
	static void gNOR(PhysXGroupsMask& results, const PhysXGroupsMask& mask0, const PhysXGroupsMask& mask1)
	{
		results.bits0 = PxU16(~(mask0.bits0 | mask1.bits0));
		results.bits1 = PxU16(~(mask0.bits1 | mask1.bits1));
		results.bits2 = PxU16(~(mask0.bits2 | mask1.bits2));
		results.bits3 = PxU16(~(mask0.bits3 | mask1.bits3));
	}
	static void gNXOR(PhysXGroupsMask& results, const PhysXGroupsMask& mask0, const PhysXGroupsMask& mask1)
	{
		results.bits0 = PxU16(~(mask0.bits0 ^ mask1.bits0));
		results.bits1 = PxU16(~(mask0.bits1 ^ mask1.bits1));
		results.bits2 = PxU16(~(mask0.bits2 ^ mask1.bits2));
		results.bits3 = PxU16(~(mask0.bits3 ^ mask1.bits3));
	}

	static void gSWAP_AND(PhysXGroupsMask& results, const PhysXGroupsMask& mask0, const PhysXGroupsMask& mask1)
	{
		results.bits0 = PxU16(mask0.bits0 & mask1.bits2);
		results.bits1 = PxU16(mask0.bits1 & mask1.bits3);
		results.bits2 = PxU16(mask0.bits2 & mask1.bits0);
		results.bits3 = PxU16(mask0.bits3 & mask1.bits1);
	}

	typedef void	(*FilterFunction)	(PhysXGroupsMask& results, const PhysXGroupsMask& mask0, const PhysXGroupsMask& mask1);

	FilterFunction const gTable[] = { gAND, gOR, gXOR, gNAND, gNOR, gNXOR, gSWAP_AND };

	static physx::PxFilterData convert(const PhysXGroupsMask& mask)
	{
		physx::PxFilterData fd;

		fd.word2 = PxU32(mask.bits0 | (mask.bits1 << 16));
		fd.word3 = PxU32(mask.bits2 | (mask.bits3 << 16));

		return fd;
	}

	static PhysXGroupsMask convert(const physx::PxFilterData& fd)
	{
		PhysXGroupsMask mask;

		mask.bits0 = PxU16((fd.word2 & 0xffff));
		mask.bits1 = PxU16((fd.word2 >> 16));
		mask.bits2 = PxU16((fd.word3 & 0xffff));
		mask.bits3 = PxU16((fd.word3 >> 16));

		return mask;
	}

	static bool getFilterData(const physx::PxActor& actor, physx::PxFilterData& fd)
	{
		physx::PxActorType::Enum aType = actor.getType();
		switch (aType)
		{
		case physx::PxActorType::eRIGID_DYNAMIC:
		case physx::PxActorType::eRIGID_STATIC:
		case physx::PxActorType::eARTICULATION_LINK:
		{
			const PxRigidActor& rActor = static_cast<const PxRigidActor&>(actor);

			PxShape* shape = NULL;
			rActor.getShapes(&shape, 1);

			fd = shape->getSimulationFilterData();
		}
		break;

		case physx::PxActorType::eACTOR_COUNT:
		case physx::PxActorType::eACTOR_FORCE_DWORD:
			break;
		}

		return true;
	}

	PX_FORCE_INLINE static void adjustFilterData(bool groupsMask, const physx::PxFilterData& src, physx::PxFilterData& dst)
	{
		if (groupsMask)
		{
			dst.word2 = src.word2;
			dst.word3 = src.word3;
		}
		else
			dst.word0 = src.word0;
	}

	template<bool TGroupsMask>
	static void setFilterData(physx::PxActor& actor, const physx::PxFilterData& fd)
	{
		physx::PxActorType::Enum aType = actor.getType();
		switch (aType)
		{
		case physx::PxActorType::eRIGID_DYNAMIC:
		case physx::PxActorType::eRIGID_STATIC:
		case physx::PxActorType::eARTICULATION_LINK:
		{
			const PxRigidActor& rActor = static_cast<const PxRigidActor&>(actor);

			PxShape* shape;
			for(PxU32 i=0; i < rActor.getNbShapes(); i++)
			{
				rActor.getShapes(&shape, 1, i);

				// retrieve current group mask
				PxFilterData resultFd = shape->getSimulationFilterData();

				adjustFilterData(TGroupsMask, fd, resultFd);

				// set new filter data
				shape->setSimulationFilterData(resultFd);
			}
		}
		break;
		case physx::PxActorType::eACTOR_COUNT:
		case physx::PxActorType::eACTOR_FORCE_DWORD:
			break;
		}
	}
}

using namespace pragma::physics;
physx::PxFilterFlags pragma::physics::PhysXSimulationFilterShader(
	physx::PxFilterObjectAttributes attributes0,
	physx::PxFilterData filterData0, 
	physx::PxFilterObjectAttributes attributes1,
	physx::PxFilterData filterData1,
	physx::PxPairFlags& pairFlags,
	const void* constantBlock,
	physx::PxU32 constantBlockSize)
{
	PX_UNUSED(constantBlock);
	PX_UNUSED(constantBlockSize);

	// let triggers through
	if(PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
	{
		pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
		return PxFilterFlags();
	}

	// Collision Group
	/*if (!gCollisionTable[filterData0.word0][filterData1.word0]())
	{
		return PxFilterFlag::eSUPPRESS;
	}*/

	if(PX_FILTER_SHOULD_PASS(filterData0,filterData1) == false)
		return physx::PxFilterFlag::eSUPPRESS;

	// Filter function
	pragma::physics::PhysXGroupsMask g0 = convert(filterData0);
	pragma::physics::PhysXGroupsMask g1 = convert(filterData1);

	pragma::physics::PhysXGroupsMask g0k0;	gTable[gFilterOps[0]](g0k0, g0, gFilterConstants[0]);
	pragma::physics::PhysXGroupsMask g1k1;	gTable[gFilterOps[1]](g1k1, g1, gFilterConstants[1]);
	pragma::physics::PhysXGroupsMask final;	gTable[gFilterOps[2]](final, g0k0, g1k1);

	bool r = final.bits0 || final.bits1 || final.bits2 || final.bits3;
	if (r != gFilterBool)
	{
		return PxFilterFlag::eSUPPRESS;
	}

	pairFlags = PxPairFlag::eCONTACT_DEFAULT;

	return PxFilterFlags();
}
/*
bool pragma::physics::PhysXGetGroupCollisionFlag(const PxU16 group1, const PxU16 group2)
{
	// PX_CHECK_AND_RETURN_NULL(group1 < 32 && group2 < 32, "Group must be less than 32");	

	return gCollisionTable[group1][group2]();
}

void pragma::physics::PhysXSetGroupCollisionFlag(const PxU16 group1, const PxU16 group2, const bool enable)
{
	// PX_CHECK_AND_RETURN(group1 < 32 && group2 < 32, "Group must be less than 32");	

	gCollisionTable[group1][group2] = enable;
	gCollisionTable[group2][group1] = enable;
}
*/
PxU16 pragma::physics::PhysXGetGroup(const physx::PxActor& actor)
{
	PxFilterData fd;
	getFilterData(actor, fd);
	return PxU16(fd.word0);
}

void pragma::physics::PhysXSetGroup(physx::PxActor& actor, const PxU16 collisionGroup)
{
	// PX_CHECK_AND_RETURN(collisionGroup < 32,"Collision group must be less than 32");
	PxFilterData fd(collisionGroup, 0, 0, 0);
	setFilterData<false>(actor, fd);
}

void pragma::physics::PhysXGetFilterOps(pragma::physics::PhysXFilterOp::Enum& op0, pragma::physics::PhysXFilterOp::Enum& op1, pragma::physics::PhysXFilterOp::Enum& op2)
{
	op0 = gFilterOps[0];
	op1 = gFilterOps[1];
	op2 = gFilterOps[2];
}

void pragma::physics::PhysXSetFilterOps(const pragma::physics::PhysXFilterOp::Enum& op0, const pragma::physics::PhysXFilterOp::Enum& op1, const pragma::physics::PhysXFilterOp::Enum& op2)
{
	gFilterOps[0] = op0;
	gFilterOps[1] = op1;
	gFilterOps[2] = op2;
}

bool pragma::physics::PhysXGetFilterBool()
{
	return gFilterBool;
}

void pragma::physics::PhysXSetFilterBool(const bool enable)
{
	gFilterBool = enable;
}

void pragma::physics::PhysXGetFilterConstants(PhysXGroupsMask& c0, PhysXGroupsMask& c1)
{
	c0 = gFilterConstants[0];
	c1 = gFilterConstants[1];
}

void pragma::physics::PhysXSetFilterConstants(const PhysXGroupsMask& c0, const PhysXGroupsMask& c1)
{
	gFilterConstants[0] = c0;
	gFilterConstants[1] = c1;
}

PhysXGroupsMask pragma::physics::PhysXGetGroupsMask(const physx::PxActor& actor)
{
	PxFilterData fd;
	if (getFilterData(actor, fd))
		return convert(fd);
	else
		return PhysXGroupsMask();
}

void pragma::physics::PhysXSetGroupsMask(physx::PxActor& actor, const PhysXGroupsMask& mask)
{
	PxFilterData fd = convert(mask);
	setFilterData<true>(actor, fd);
}
#pragma optimize("",on)
