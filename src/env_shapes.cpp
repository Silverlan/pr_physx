#include "pr_physx/environment.hpp"
#include "pr_physx/shape.hpp"
#include "pr_physx/material.hpp"
#include <pragma/math/surfacematerial.h>
#include <pragma/networkstate/networkstate.h>

void pragma::physics::PhysXEnvironment::InitializeShape(PhysXActorShape &shape,bool basicOnly) const
{
	physx::PxTransform t {physx::PxVec3{0.f,0.f,0.f},physx::PxQuat{1.f}};

	auto &actorShape = shape.GetActorShape();
	actorShape.userData = &shape;
	actorShape.setFlag(physx::PxShapeFlag::eSIMULATION_SHAPE,true);
	actorShape.setFlag(physx::PxShapeFlag::eVISUALIZATION,true);
	if(basicOnly)
		return;
	actorShape.setLocalPose(t);
}
std::shared_ptr<pragma::physics::IConvexShape> pragma::physics::PhysXEnvironment::CreateCapsuleShape(float halfWidth,float halfHeight,const IMaterial &mat)
{
	auto geometry = std::shared_ptr<physx::PxGeometry>(
		new physx::PxCapsuleGeometry{static_cast<physx::PxReal>(ToPhysXLength(halfWidth)),static_cast<physx::PxReal>(ToPhysXLength(halfHeight))},
		[](physx::PxGeometry *p) {delete p;}
	);
	if(geometry == nullptr)
		return nullptr;
	return CreateSharedPtr<PhysXConvexShape>(*this,geometry);
}
std::shared_ptr<pragma::physics::IConvexShape> pragma::physics::PhysXEnvironment::CreateBoxShape(const Vector3 &halfExtents,const IMaterial &mat)
{
	auto geometry = std::shared_ptr<physx::PxGeometry>(
		new physx::PxBoxGeometry{ToPhysXVector(halfExtents)},
		[](physx::PxGeometry *p) {delete p;}
	);
	if(geometry == nullptr)
		return nullptr;
	return CreateSharedPtr<PhysXConvexShape>(*this,geometry);
}
std::shared_ptr<pragma::physics::IConvexShape> pragma::physics::PhysXEnvironment::CreateCylinderShape(float radius,float height,const IMaterial &mat)
{
	// TODO
	return nullptr;
}
std::shared_ptr<pragma::physics::IConvexShape> pragma::physics::PhysXEnvironment::CreateSphereShape(float radius,const IMaterial &mat)
{
	auto geometry = std::shared_ptr<physx::PxGeometry>(
		new physx::PxSphereGeometry{static_cast<float>(ToPhysXLength(radius))},
		[](physx::PxGeometry *p) {delete p;}
	);
	auto pxShape = px_create_unique_ptr(GetPhysics().createShape(*geometry,dynamic_cast<const PhysXMaterial&>(mat).GetInternalObject(),false));
	if(pxShape == nullptr)
		return nullptr;
	return CreateSharedPtr<PhysXConvexShape>(*this,geometry);
}
std::shared_ptr<pragma::physics::IConvexHullShape> pragma::physics::PhysXEnvironment::CreateConvexHullShape(const IMaterial &mat)
{
	auto shape = CreateSharedPtr<PhysXConvexHullShape>(*this);
	shape->SetMaterial(mat);
	// InitializeShape(*shape); // Will be initialized once the shape has been built!
	return shape;
}
std::shared_ptr<pragma::physics::ITriangleShape> pragma::physics::PhysXEnvironment::CreateTriangleShape(const IMaterial &mat)
{
	auto shape = CreateSharedPtr<PhysXTriangleShape>(*this);
	shape->SetMaterial(mat);
	// InitializeShape(*shape); // Will be initialized once the shape has been built!
	return shape;
}
std::shared_ptr<pragma::physics::ICompoundShape> pragma::physics::PhysXEnvironment::CreateCompoundShape(std::vector<IShape*> &shapes)
{
	auto shape = CreateSharedPtr<PhysXCompoundShape>(*this);
	for(auto *subShape : shapes)
		shape->AddShape(*subShape);
	// InitializeShape(*shape); // No initialization required
	return shape;
}
std::shared_ptr<pragma::physics::IShape> pragma::physics::PhysXEnvironment::CreateHeightfieldTerrainShape(uint32_t width,uint32_t length,Scalar maxHeight,uint32_t upAxis,const IMaterial &mat)
{
	return nullptr;
}
