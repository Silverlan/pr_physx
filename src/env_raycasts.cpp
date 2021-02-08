/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "pr_physx/environment.hpp"
#include "pr_physx/collision_object.hpp"
#include "pr_physx/raycast.hpp"
#include "pr_physx/shape.hpp"
#include <pragma/entities/baseentity.h>
#include <pragma/physics/raytraces.h>

void pragma::physics::PhysXEnvironment::InitializeRayCastResult(const TraceData &data,float rayLength,const physx::PxRaycastHit &raycastHit,TraceResult &outResult,RayCastHitType hitType) const
{
	auto *colObj = raycastHit.actor ? GetCollisionObject(*raycastHit.actor) : nullptr;
	if(colObj)
	{
		outResult.collisionObj = util::weak_shared_handle_cast<IBase,ICollisionObject>(colObj->GetHandle());
		auto *physObj = colObj->GetPhysObj();
		if(physObj)
		{
			outResult.physObj = physObj->GetHandle();
			auto *ent = outResult.physObj->GetOwner();
			outResult.entity = ent ? ent->GetEntity().GetHandle() : EntityHandle{};
		}
	}
	if(raycastHit.shape)
	{
		auto &shape = *GetShape(*raycastHit.shape);
		outResult.shape = std::static_pointer_cast<IShape>(shape.GetShape().shared_from_this());
	}
	outResult.distance = FromPhysXLength(raycastHit.distance);
	outResult.fraction = rayLength /outResult.distance;
	outResult.hitType = hitType;
	outResult.normal = FromPhysXNormal(raycastHit.normal);
	outResult.position = FromPhysXVector(raycastHit.position);
	outResult.startPosition = data.GetSourceOrigin();
}
void pragma::physics::PhysXEnvironment::InitializeRayCastResult(const TraceData &data,float rayLength,const physx::PxOverlapHit &raycastHit,TraceResult &outResult,RayCastHitType hitType) const
{
	auto *colObj = raycastHit.actor ? GetCollisionObject(*raycastHit.actor) : nullptr;
	if(colObj)
	{
		outResult.collisionObj = util::weak_shared_handle_cast<IBase,ICollisionObject>(colObj->GetHandle());
		auto *physObj = colObj->GetPhysObj();
		if(physObj)
		{
			outResult.physObj = physObj->GetHandle();
			auto *ent = outResult.physObj->GetOwner();
			outResult.entity = ent ? ent->GetEntity().GetHandle() : EntityHandle{};
		}
	}
	if(raycastHit.shape)
	{
		auto &shape = *GetShape(*raycastHit.shape);
		outResult.shape = std::static_pointer_cast<IShape>(shape.GetShape().shared_from_this());
	}
	outResult.hitType = hitType;
	outResult.startPosition = data.GetSourceOrigin();
}
void pragma::physics::PhysXEnvironment::InitializeRayCastResult(const TraceData &data,float rayLength,const physx::PxSweepHit &raycastHit,TraceResult &outResult,RayCastHitType hitType) const
{
	auto *colObj = raycastHit.actor ? GetCollisionObject(*raycastHit.actor) : nullptr;
	if(colObj)
	{
		outResult.collisionObj = util::weak_shared_handle_cast<IBase,ICollisionObject>(colObj->GetHandle());
		auto *physObj = colObj->GetPhysObj();
		if(physObj)
		{
			outResult.physObj = physObj->GetHandle();
			auto *ent = outResult.physObj->GetOwner();
			outResult.entity = ent ? ent->GetEntity().GetHandle() : EntityHandle{};
		}
	}
	if(raycastHit.shape)
	{
		auto &shape = *GetShape(*raycastHit.shape);
		outResult.shape = std::static_pointer_cast<IShape>(shape.GetShape().shared_from_this());
	}
	outResult.distance = FromPhysXLength(raycastHit.distance);
	outResult.fraction = rayLength /outResult.distance;
	outResult.hitType = hitType;
	outResult.normal = FromPhysXNormal(raycastHit.normal);
	outResult.position = FromPhysXVector(raycastHit.position);
	outResult.startPosition = data.GetSourceOrigin();
}

static std::unique_ptr<pragma::physics::RayCastFilterCallback> get_raycast_filter(const pragma::physics::PhysXEnvironment &env,const TraceData &data,physx::PxHitFlags &hitFlags,physx::PxQueryFilterData &queryFilterData)
{
	auto flags = data.GetFlags();
	physx::PxQueryFlags queryFlags = physx::PxQueryFlag::eDYNAMIC | physx::PxQueryFlag::eSTATIC;
	hitFlags = static_cast<physx::PxHitFlags>(0);
	if(umath::is_flag_set(flags,RayCastFlags::ReportHitPosition))
		hitFlags |= physx::PxHitFlag::ePOSITION;
	if(umath::is_flag_set(flags,RayCastFlags::ReportHitNormal))
		hitFlags |= physx::PxHitFlag::eNORMAL;
	if(umath::is_flag_set(flags,RayCastFlags::ReportHitUV))
		hitFlags |= physx::PxHitFlag::eUV;
	if(umath::is_flag_set(flags,RayCastFlags::ReportAllResults))
		hitFlags |= physx::PxHitFlag::eMESH_MULTIPLE;
	if(umath::is_flag_set(flags,RayCastFlags::ReportAnyResult))
	{
		queryFlags = physx::PxQueryFlag::eANY_HIT;
		hitFlags |= physx::PxHitFlag::eMESH_ANY;
	}
	if(umath::is_flag_set(flags,RayCastFlags::ReportBackFaceHits))
		hitFlags |= physx::PxHitFlag::eMESH_BOTH_SIDES;
	if(umath::is_flag_set(flags,RayCastFlags::Precise))
		hitFlags |= physx::PxHitFlag::ePRECISE_SWEEP;

	if(umath::is_flag_set(flags,RayCastFlags::IgnoreDynamic))
		queryFlags &= ~physx::PxQueryFlag::eDYNAMIC;
	if(umath::is_flag_set(flags,RayCastFlags::IgnoreStatic))
		queryFlags &= ~physx::PxQueryFlag::eSTATIC;

	auto &filter = data.GetFilter();
	std::unique_ptr<pragma::physics::RayCastFilterCallback> pxFilter = nullptr;
	if(filter)
	{
		pxFilter = std::make_unique<pragma::physics::RayCastFilterCallback>(env,*filter,umath::is_flag_set(flags,RayCastFlags::InvertFilter));
		if(filter->HasPreFilter())
			queryFlags |= physx::PxQueryFlag::ePREFILTER;
		if(filter->HasPostFilter())
			queryFlags |= physx::PxQueryFlag::ePOSTFILTER;
	}
	queryFilterData = physx::PxQueryFilterData{queryFlags};
	return pxFilter;
}
Bool pragma::physics::PhysXEnvironment::Overlap(const TraceData &data,std::vector<TraceResult> *optOutResults) const
{
	auto *shape = data.GetShape();
	if(shape == nullptr || shape->IsConvex() == false)
		return false;
	auto &convexShape = static_cast<const PhysXConvexShape&>(PhysXShape::GetShape(*shape));
	if(convexShape.m_geometry == nullptr)
		return false;
	physx::PxTransform pose {
		ToPhysXVector(data.GetSourceOrigin()),
		ToPhysXRotation(data.GetSourceRotation())
	};
	auto origin = ToPhysXVector(data.GetSourceOrigin());
	auto target = data.GetTargetOrigin();
	auto distance = uvec::length(target);
	if(distance == 0.f)
		return false;
	auto unitDir = ToPhysXVector(target);
	unitDir /= distance;

	auto hitFlags = static_cast<physx::PxHitFlags>(0);
	physx::PxQueryFilterData queryFilterData {};
	auto pxFilter = get_raycast_filter(*this,data,hitFlags,queryFilterData);

	physx::PxOverlapBuffer hit {};
	std::array<physx::PxOverlapHit,32> touchingHits; // Arbitrary maximum number of touches
	hit.touches = touchingHits.data();
	hit.maxNbTouches = touchingHits.size();
	auto bHitAny = m_scene->overlap(*convexShape.m_geometry,pose,hit,queryFilterData,pxFilter.get());
	if(optOutResults == nullptr || bHitAny == false)
		return bHitAny;
	auto numTouches = hit.getNbTouches();
	optOutResults->reserve(numTouches +1);
	for(auto i=decltype(numTouches){0u};i<numTouches;++i)
	{
		auto &touchHit = hit.getTouch(i);
		optOutResults->push_back({});
		auto &result = optOutResults->back();
		InitializeRayCastResult(data,distance,touchHit,result,RayCastHitType::Touch);
	}
	optOutResults->push_back({});
	auto &result = optOutResults->back();
	InitializeRayCastResult(data,distance,hit.block,result,hit.hasBlock ? RayCastHitType::Block : RayCastHitType::None);
	return bHitAny;
}

Bool pragma::physics::PhysXEnvironment::RayCast(const TraceData &data,std::vector<TraceResult> *optOutResults) const
{
	auto origin = ToPhysXVector(data.GetSourceOrigin());
	auto target = ToPhysXVector(data.GetTargetOrigin());
	auto unitDir = target -origin;
	auto distance = unitDir.magnitude();
	if(distance == 0.f)
		return false;
	unitDir /= distance;

	auto hitFlags = static_cast<physx::PxHitFlags>(0);
	physx::PxQueryFilterData queryFilterData {};
	auto pxFilter = get_raycast_filter(*this,data,hitFlags,queryFilterData);

	physx::PxRaycastBuffer hit {};
	std::array<physx::PxRaycastHit,32> touchingHits; // Arbitrary maximum number of touches
	hit.touches = touchingHits.data();
	hit.maxNbTouches = touchingHits.size();
	auto bHitAny = m_scene->raycast(origin,unitDir,distance,hit,hitFlags,queryFilterData,pxFilter.get());
	if(optOutResults == nullptr || bHitAny == false)
		return bHitAny;
	auto numTouches = hit.getNbTouches();
	optOutResults->reserve(numTouches +1);
	for(auto i=decltype(numTouches){0u};i<numTouches;++i)
	{
		auto &touchHit = hit.getTouch(i);
		optOutResults->push_back({});
		auto &result = optOutResults->back();
		InitializeRayCastResult(data,distance,touchHit,result,RayCastHitType::Touch);
	}
	optOutResults->push_back({});
	auto &result = optOutResults->back();
	InitializeRayCastResult(data,distance,hit.block,result,hit.hasBlock ? RayCastHitType::Block : RayCastHitType::None);
	return bHitAny;
}
Bool pragma::physics::PhysXEnvironment::Sweep(const TraceData &data,std::vector<TraceResult> *optOutResults) const
{
	auto *shape = data.GetShape();
	if(shape == nullptr || shape->IsConvex() == false)
		return false;
	auto &convexShape = static_cast<const PhysXConvexShape&>(PhysXShape::GetShape(*shape));
	if(convexShape.m_geometry == nullptr)
		return false;
	physx::PxTransform pose {
		ToPhysXVector(data.GetSourceOrigin()),
		ToPhysXRotation(data.GetSourceRotation())
	};
	auto target = data.GetTargetOrigin();
	auto distance = uvec::length(target);
	if(distance == 0.f)
		return false;
	auto unitDir = ToPhysXVector(target);
	unitDir /= distance;

	auto hitFlags = static_cast<physx::PxHitFlags>(0);
	physx::PxQueryFilterData queryFilterData {};
	auto pxFilter = get_raycast_filter(*this,data,hitFlags,queryFilterData);

	physx::PxSweepBuffer hit {};
	std::array<physx::PxSweepHit,32> touchingHits; // Arbitrary maximum number of touches
	hit.touches = touchingHits.data();
	hit.maxNbTouches = touchingHits.size();
	auto bHitAny = m_scene->sweep(*convexShape.m_geometry,pose,unitDir,distance,hit,hitFlags,queryFilterData,pxFilter.get());
	if(optOutResults == nullptr || bHitAny == false)
		return bHitAny;
	auto numTouches = hit.getNbTouches();
	optOutResults->reserve(numTouches +1);
	for(auto i=decltype(numTouches){0u};i<numTouches;++i)
	{
		auto &touchHit = hit.getTouch(i);
		optOutResults->push_back({});
		auto &result = optOutResults->back();
		InitializeRayCastResult(data,distance,touchHit,result,RayCastHitType::Touch);
	}
	optOutResults->push_back({});
	auto &result = optOutResults->back();
	InitializeRayCastResult(data,distance,hit.block,result,hit.hasBlock ? RayCastHitType::Block : RayCastHitType::None);
	return bHitAny;
}
physx::PxCooking &pragma::physics::PhysXEnvironment::GetCooking() {return *m_cooking;}
