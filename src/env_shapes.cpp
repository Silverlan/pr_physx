#include "pr_physx/environment.hpp"
#include "pr_physx/shape.hpp"
#include "pr_physx/material.hpp"
#include <pragma/math/surfacematerial.h>
#include <pragma/networkstate/networkstate.h>

void pragma::physics::PxEnvironment::InitializeShape(PxShape &shape,bool basicOnly)
{
	shape.GetInternalObject().userData = &shape;
	if(basicOnly)
		return;
	shape.GetInternalObject().setFlag(physx::PxShapeFlag::eSIMULATION_SHAPE,true);
	shape.GetInternalObject().setFlag(physx::PxShapeFlag::eVISUALIZATION,true);

	physx::PxTransform t {physx::PxVec3{0.f,0.f,0.f},physx::PxQuat{1.f}};
	shape.GetInternalObject().setLocalPose(t);
}
std::shared_ptr<pragma::physics::IConvexShape> pragma::physics::PxEnvironment::CreateCapsuleShape(float halfWidth,float halfHeight,const IMaterial &mat)
{
	auto geometry = std::unique_ptr<physx::PxGeometry,void(*)(physx::PxGeometry*)>(
		new physx::PxCapsuleGeometry{static_cast<physx::PxReal>(ToPhysXLength(halfWidth)),static_cast<physx::PxReal>(ToPhysXLength(halfHeight))},
		[](physx::PxGeometry *p) {delete p;}
	);
	auto pxShape = px_create_unique_ptr(GetPhysics().createShape(*geometry,dynamic_cast<const PxMaterial&>(mat).GetInternalObject(),false));
	if(pxShape == nullptr)
		return nullptr;
	auto shape = CreateSharedPtr<PxConvexShape>(*this,std::move(pxShape),std::move(geometry));
	InitializeShape(*shape);
	return shape;
}
std::shared_ptr<pragma::physics::IConvexShape> pragma::physics::PxEnvironment::CreateBoxShape(const Vector3 &halfExtents,const IMaterial &mat)
{
	auto geometry = std::unique_ptr<physx::PxGeometry,void(*)(physx::PxGeometry*)>(
		new physx::PxBoxGeometry{ToPhysXVector(halfExtents)},
		[](physx::PxGeometry *p) {delete p;}
	);
	auto pxShape = px_create_unique_ptr(GetPhysics().createShape(*geometry,dynamic_cast<const PxMaterial&>(mat).GetInternalObject(),false));
	if(pxShape == nullptr)
		return nullptr;
	auto shape = CreateSharedPtr<PxConvexShape>(*this,std::move(pxShape),std::move(geometry));
	InitializeShape(*shape);
	return shape;
}
std::shared_ptr<pragma::physics::IConvexShape> pragma::physics::PxEnvironment::CreateCylinderShape(float radius,float height,const IMaterial &mat)
{
	// TODO
	return nullptr;
}
std::shared_ptr<pragma::physics::IConvexShape> pragma::physics::PxEnvironment::CreateSphereShape(float radius,const IMaterial &mat)
{
	auto geometry = std::unique_ptr<physx::PxGeometry,void(*)(physx::PxGeometry*)>(
		new physx::PxSphereGeometry{static_cast<float>(ToPhysXLength(radius))},
		[](physx::PxGeometry *p) {delete p;}
	);
	auto pxShape = px_create_unique_ptr(GetPhysics().createShape(*geometry,dynamic_cast<const PxMaterial&>(mat).GetInternalObject(),false));
	if(pxShape == nullptr)
		return nullptr;
	auto shape = CreateSharedPtr<PxConvexShape>(*this,std::move(pxShape),std::move(geometry));
	InitializeShape(*shape);
	return shape;
}
std::shared_ptr<pragma::physics::IConvexHullShape> pragma::physics::PxEnvironment::CreateConvexHullShape(const IMaterial &mat)
{
	auto shape = CreateSharedPtr<PxConvexHullShape>(*this);
	auto *surfMat = mat.GetSurfaceMaterial();
	if(surfMat)
		shape->SetSurfaceMaterial(surfMat->GetIndex());
	// InitializeShape(*shape); // Will be initialized once the shape has been built!
	return shape;
}
std::shared_ptr<pragma::physics::ITriangleShape> pragma::physics::PxEnvironment::CreateTriangleShape(const IMaterial &mat)
{
	if(true)
		return nullptr;
	auto shape = CreateSharedPtr<PxTriangleShape>(*this);
	auto *surfMat = mat.GetSurfaceMaterial();
	if(surfMat)
		shape->SetSurfaceMaterial(surfMat->GetIndex());
	// InitializeShape(*shape); // Will be initialized once the shape has been built!
	return shape;
}
std::shared_ptr<pragma::physics::ICompoundShape> pragma::physics::PxEnvironment::CreateCompoundShape(std::vector<IShape*> &shapes)
{
	auto shape = CreateSharedPtr<PxCompoundShape>(*this);
	for(auto *subShape : shapes)
		shape->AddShape(*subShape);
	// InitializeShape(*shape); // No initialization required
	return shape;
}
std::shared_ptr<pragma::physics::IShape> pragma::physics::PxEnvironment::CreateHeightfieldTerrainShape(uint32_t width,uint32_t length,Scalar maxHeight,uint32_t upAxis,const IMaterial &mat)
{
	return nullptr;
}
