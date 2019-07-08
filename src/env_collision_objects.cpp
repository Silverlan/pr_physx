#include "pr_physx/environment.hpp"
#include "pr_physx/shape.hpp"
#include "pr_physx/collision_object.hpp"
#include <pragma/networkstate/networkstate.h>

void pragma::physics::PxEnvironment::InitializeCollisionObject(PxCollisionObject &o)
{
	o.GetInternalObject().setActorFlag(physx::PxActorFlag::eVISUALIZATION,true);
}
util::TSharedHandle<pragma::physics::ICollisionObject> pragma::physics::PxEnvironment::CreateCollisionObject(IShape &shape)
{
	return nullptr;
}
util::TSharedHandle<pragma::physics::IRigidBody> pragma::physics::PxEnvironment::CreateRigidBody(float mass,IShape &shape,const Vector3 &localInertia,bool dynamic)
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
	auto *pRigidObj = static_cast<physx::PxRigidActor*>(rigidObj.get());
	auto rigidBody = dynamic ? util::shared_handle_cast<PxRigidDynamic,PxRigidBody>(CreateSharedHandle<PxRigidDynamic>(*this,std::move(rigidObj),shape,mass,localInertia)) :
		util::shared_handle_cast<PxRigidStatic,PxRigidBody>(CreateSharedHandle<PxRigidStatic>(*this,std::move(rigidObj),shape,mass,localInertia));
	if(dynamic && (shape.IsTriangleShape() || shape.IsHeightfield())) // Dynamic rigid bodies with triangle or height field shape HAVE to be kinematic
		static_cast<physx::PxRigidDynamic*>(pRigidObj)->setRigidBodyFlag(physx::PxRigidBodyFlag::eKINEMATIC,true);
	if(pRigidObj->attachShape(const_cast<PxShape&>(dynamic_cast<const PxShape&>(shape)).GetInternalObject()) == false)
		return nullptr;

	physx::PxFilterData filterData;
	filterData.word0 = 1; // Group // word0 = own ID
	filterData.word1 = 0; // Mask  // word1 = ID mask to filter pairs that trigger a
	filterData.word2 = 0;
	filterData.word3 = 0;
	PxShape::GetShape(shape).GetInternalObject().setSimulationFilterData(filterData);

	auto density = 1.f; // TODO
	if(dynamic)
		physx::PxRigidBodyExt::updateMassAndInertia(*static_cast<physx::PxRigidDynamic*>(pRigidObj),density);
	InitializeCollisionObject(*rigidBody);
	AddCollisionObject(*rigidBody);
	return util::shared_handle_cast<PxRigidBody,IRigidBody>(rigidBody);
}
util::TSharedHandle<pragma::physics::ISoftBody> pragma::physics::PxEnvironment::CreateSoftBody(const PhysSoftBodyInfo &info,float mass,const std::vector<Vector3> &verts,const std::vector<uint16_t> &indices,std::vector<uint16_t> &indexTranslations)
{
	return nullptr;
}
util::TSharedHandle<pragma::physics::IGhostObject> pragma::physics::PxEnvironment::CreateGhostObject(IShape &shape)
{
	return nullptr;
}
