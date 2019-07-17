#include "pr_physx/environment.hpp"
#include "pr_physx/shape.hpp"
#include "pr_physx/material.hpp"
#include "pr_physx/collision_object.hpp"
#include <pragma/networkstate/networkstate.h>

void pragma::physics::PhysXEnvironment::InitializeCollisionObject(PhysXCollisionObject &o)
{
	o.GetInternalObject().setActorFlag(physx::PxActorFlag::eVISUALIZATION,true);
}
util::TSharedHandle<pragma::physics::ICollisionObject> pragma::physics::PhysXEnvironment::CreatePlane(const Vector3 &n,float d,const IMaterial &mat)
{
	physx::PxPlane plane {
		n.x,n.y,n.z,
		static_cast<float>(ToPhysXLength(d))
	};
	auto rigidStaticPlane = px_create_unique_ptr<physx::PxActor>(PxCreatePlane(GetPhysics(),plane,const_cast<physx::PxMaterial&>(PhysXMaterial::GetMaterial(mat).GetInternalObject())));
	auto *pRigidStaticPlane = static_cast<physx::PxRigidStatic*>(rigidStaticPlane.get());
	if(rigidStaticPlane == nullptr)
		return nullptr;
	physx::PxShape *pxShape;
	pRigidStaticPlane->getShapes(&pxShape,1);
	if(pxShape == nullptr)
		return nullptr;
	auto &pxPlaneGeometry = pxShape->getGeometry().plane();
	auto geometryPtr = std::shared_ptr<physx::PxGeometry>{&pxPlaneGeometry,[](physx::PxGeometry*) {}};
	if(geometryPtr == nullptr)
		return nullptr;
	auto shape = CreateSharedPtr<PhysXConvexShape>(*this,geometryPtr);
	auto rigidBody = CreateSharedHandle<PhysXRigidStatic>(*this,std::move(rigidStaticPlane),*shape);
	rigidBody->GetActorShapeCollection().AddShape(*shape,*pxShape);
	InitializeCollisionObject(*rigidBody);
	AddCollisionObject(*rigidBody);
	return util::shared_handle_cast<PhysXRigidStatic,ICollisionObject>(rigidBody);
}
util::TSharedHandle<pragma::physics::ICollisionObject> pragma::physics::PhysXEnvironment::CreateCollisionObject(IShape &shape)
{
	return nullptr;
}
util::TSharedHandle<pragma::physics::IRigidBody> pragma::physics::PhysXEnvironment::CreateRigidBody(IShape &shape,bool dynamic)
{
	if(shape.IsValid() == false)
		return nullptr;
	physx::PxTransform t {physx::PxVec3{0.f,0.f,0.f},physx::PxQuat{1.f}};
	auto rigidObj = px_create_unique_ptr<physx::PxActor>(
		dynamic ? static_cast<physx::PxRigidActor*>(GetPhysics().createRigidDynamic(t)) :
		static_cast<physx::PxRigidActor*>(GetPhysics().createRigidStatic(t))
	);
	if(rigidObj == nullptr)
		return nullptr;
	if(dynamic)
		static_cast<physx::PxRigidDynamic*>(rigidObj.get())->setMass(shape.GetMass());
	auto *pRigidObj = static_cast<physx::PxRigidActor*>(rigidObj.get());
	auto rigidBody = dynamic ? util::shared_handle_cast<PhysXRigidDynamic,PhysXRigidBody>(CreateSharedHandle<PhysXRigidDynamic>(*this,std::move(rigidObj),shape)) :
		util::shared_handle_cast<PhysXRigidStatic,PhysXRigidBody>(CreateSharedHandle<PhysXRigidStatic>(*this,std::move(rigidObj),shape));
	if(dynamic && (shape.IsTriangleShape() || shape.IsHeightfield())) // Dynamic rigid bodies with triangle or height field shape HAVE to be kinematic
		static_cast<physx::PxRigidDynamic*>(pRigidObj)->setRigidBodyFlag(physx::PxRigidBodyFlag::eKINEMATIC,true);
	rigidBody->SetCollisionShape(&shape);

	if(dynamic)
		physx::PxRigidBodyExt::setMassAndUpdateInertia(*static_cast<physx::PxRigidDynamic*>(pRigidObj),shape.GetMass());
	InitializeCollisionObject(*rigidBody);
	AddCollisionObject(*rigidBody);
	return util::shared_handle_cast<PhysXRigidBody,IRigidBody>(rigidBody);
}
util::TSharedHandle<pragma::physics::ISoftBody> pragma::physics::PhysXEnvironment::CreateSoftBody(const PhysSoftBodyInfo &info,float mass,const std::vector<Vector3> &verts,const std::vector<uint16_t> &indices,std::vector<uint16_t> &indexTranslations)
{
	return nullptr;
}
util::TSharedHandle<pragma::physics::IGhostObject> pragma::physics::PhysXEnvironment::CreateGhostObject(IShape &shape)
{
	return nullptr;
}
