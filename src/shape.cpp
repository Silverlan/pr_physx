#include "pr_physx/shape.hpp"
#include "pr_physx/environment.hpp"
#include <PxShape.h>

pragma::physics::PxShape &pragma::physics::PxShape::GetShape(IShape &s)
{
	return *static_cast<PxShape*>(s.userData);
}
const pragma::physics::PxShape &pragma::physics::PxShape::GetShape(const IShape &o) {return GetShape(const_cast<IShape&>(o));}
pragma::physics::PxShape::PxShape(IEnvironment &env,PxUniquePtr<physx::PxShape> shape,PxUniquePtr<physx::PxGeometry> geometry)
	: IShape{env},m_geometry{std::move(geometry)},m_shape{std::move(shape)}
{
	userData = this;
}

const physx::PxShape &pragma::physics::PxShape::GetInternalObject() const {return *m_shape;}
physx::PxShape &pragma::physics::PxShape::GetInternalObject() {return *m_shape;}
pragma::physics::PxEnvironment &pragma::physics::PxShape::GetPxEnv() const {return static_cast<PxEnvironment&>(m_physEnv);}
void pragma::physics::PxShape::CalculateLocalInertia(float mass,Vector3 *localInertia) const
{
	// TODO
}
void pragma::physics::PxShape::GetAABB(Vector3 &min,Vector3 &max) const
{
	auto t = m_shape->getLocalPose();
	auto origin = GetPxEnv().FromPhysXVector(t.p);
	switch(m_geometry->getType())
	{
	case physx::PxGeometryType::eBOX:
		{
			auto &box = static_cast<physx::PxBoxGeometry&>(*m_geometry);
			auto extents = GetPxEnv().FromPhysXVector(box.halfExtents);
			min = origin -extents;
			max = origin +extents;
			break;
		}
	case physx::PxGeometryType::eSPHERE:
		{
			auto &sphere = static_cast<physx::PxSphereGeometry&>(*m_geometry);
			auto extents = Vector3{sphere.radius,sphere.radius,sphere.radius};
			min = origin -extents;
			max = origin +extents;
			break;
		}
	}
	// TODO
/*
enum Enum
{
ePLANE,
eCAPSULE,
eCONVEXMESH,
eTRIANGLEMESH,
eHEIGHTFIELD,
*/
}
void pragma::physics::PxShape::GetBoundingSphere(Vector3 &outCenter,float &outRadius) const
{
	// TODO
}

void pragma::physics::PxShape::SetTrigger(bool bTrigger)
{
	GetInternalObject().setFlag(physx::PxShapeFlag::eSIMULATION_SHAPE,!bTrigger);
	GetInternalObject().setFlag(physx::PxShapeFlag::eTRIGGER_SHAPE,bTrigger);
}
bool pragma::physics::PxShape::IsTrigger() const
{
	return GetInternalObject().getFlags().isSet(physx::PxShapeFlag::eTRIGGER_SHAPE);
}

//////////////

pragma::physics::PxConvexShape::PxConvexShape(IEnvironment &env,PxUniquePtr<physx::PxShape> shape,PxUniquePtr<physx::PxGeometry> geometry)
	: IConvexShape{env},PxShape{env,std::move(shape),std::move(geometry)},IShape{env}
{}
void pragma::physics::PxConvexShape::SetLocalScaling(const Vector3 &scale)
{
	// TODO
}

//////////////

pragma::physics::PxCapsuleShape::PxCapsuleShape(IEnvironment &env,PxUniquePtr<physx::PxShape> shape,PxUniquePtr<physx::PxGeometry> geometry)
	: ICapsuleShape{env},PxConvexShape{env,std::move(shape),std::move(geometry)},IShape{env},IConvexShape{env}
{}

float pragma::physics::PxCapsuleShape::GetRadius() const
{
	return GetPxEnv().FromPhysXLength(GetInternalObject().getGeometry().capsule().radius);
}
float pragma::physics::PxCapsuleShape::GetHalfHeight() const
{
	return GetPxEnv().FromPhysXLength(GetInternalObject().getGeometry().capsule().halfHeight *0.5f);
}

//////////////

pragma::physics::PxBoxShape::PxBoxShape(IEnvironment &env,PxUniquePtr<physx::PxShape> shape,PxUniquePtr<physx::PxGeometry> geometry)
	: IBoxShape{env},PxConvexShape{env,std::move(shape),std::move(geometry)},IShape{env},IConvexShape{env}
{}

Vector3 pragma::physics::PxBoxShape::GetHalfExtents() const
{
	return GetPxEnv().FromPhysXVector(GetInternalObject().getGeometry().box().halfExtents);
}
