#include "pr_physx/shape.hpp"
#include "pr_physx/environment.hpp"
#include "pr_physx/material.hpp"
#include "pr_physx/common.hpp"
#include <pragma/math/surfacematerial.h>
#include <PxShape.h>
#include <common/PxCoreUtilityTypes.h>

#pragma optimize("",off)
pragma::physics::PxShape &pragma::physics::PxShape::GetShape(IShape &s)
{
	return *static_cast<PxShape*>(s.GetUserData());
}
const pragma::physics::PxShape &pragma::physics::PxShape::GetShape(const IShape &o) {return GetShape(const_cast<IShape&>(o));}
pragma::physics::PxShape::PxShape(IEnvironment &env,PxUniquePtr<physx::PxShape> shape,PxUniquePtr<physx::PxGeometry> geometry)
	: IShape{env},m_geometry{std::move(geometry)},m_shape{std::move(shape)}
{
	SetUserData(this);
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

void pragma::physics::PxShape::SetLocalPose(const Transform &t)
{
	GetInternalObject().setLocalPose(GetPxEnv().CreatePxTransform(t));
}
pragma::physics::Transform pragma::physics::PxShape::GetLocalPose() const
{
	return GetPxEnv().CreateTransform(GetInternalObject().getLocalPose());
}

bool pragma::physics::PxShape::IsValid() const {return IShape::IsValid() && m_shape != nullptr;}

//////////////

pragma::physics::PxConvexShape::PxConvexShape(IEnvironment &env,PxUniquePtr<physx::PxShape> shape,PxUniquePtr<physx::PxGeometry> geometry)
	: IConvexShape{env},PxShape{env,std::move(shape),std::move(geometry)},IShape{env}
{}
void pragma::physics::PxConvexShape::SetLocalScaling(const Vector3 &scale)
{
	// TODO
}

//////////////

pragma::physics::PxConvexHullShape::PxConvexHullShape(IEnvironment &env)
	: IConvexHullShape{env},IShape{env},IConvexShape{env},PxConvexShape{env,px_null_ptr<physx::PxShape>(),px_null_ptr<physx::PxGeometry>()}
{}
void pragma::physics::PxConvexHullShape::AddPoint(const Vector3 &point)
{
	m_vertices.push_back(point);
}
void pragma::physics::PxConvexHullShape::Build()
{
	m_shape = nullptr;

	// TODO: Convert to PhysX coordinate system
	static_assert(sizeof(decltype(m_vertices.front())) == sizeof(physx::PxVec3),"Vertex size mismatch!");
	physx::PxConvexMeshDesc meshDesc;
	meshDesc.points.count = m_vertices.size();
	meshDesc.points.stride = sizeof(m_vertices.front());
	meshDesc.points.data = m_vertices.data();
	meshDesc.flags = physx::PxConvexFlag::e16_BIT_INDICES;

	std::vector<physx::PxHullPolygon> polys {};
	if(true)//m_triangles.empty()) // TODO
	{
		meshDesc.flags |= physx::PxConvexFlag::eCOMPUTE_CONVEX;
		meshDesc.vertexLimit = 256;
	}
	else
	{
		meshDesc.indices.data = m_triangles.data();
		meshDesc.indices.count = m_triangles.size();
		meshDesc.indices.stride = sizeof(m_triangles.front());

		polys.reserve(m_triangles.size() /3);
		for(auto i=decltype(m_triangles.size()){0u};i<m_triangles.size();i+=3)
		{
			polys.push_back({});
			auto &poly = polys.back();
			auto idx0 = m_triangles.at(i);
			auto idx1 = m_triangles.at(i +1);
			auto idx2 = m_triangles.at(i +2);

			Vector3 n;
			float d;
			uvec::calc_plane(m_vertices.at(idx0),m_vertices.at(idx1),m_vertices.at(idx2),n,d);

			auto nPx = -GetPxEnv().ToPhysXNormal(n);
			auto dPx = GetPxEnv().ToPhysXLength(d);
			poly.mNbVerts = 3;
			poly.mPlane[0] = nPx[0];
			poly.mPlane[1] = nPx[1];
			poly.mPlane[2] = nPx[2];
			poly.mPlane[3] = dPx;
			poly.mIndexBase = i;
		}
		meshDesc.polygons.count = polys.size();
		meshDesc.polygons.stride = sizeof(polys.front());
		meshDesc.polygons.data = polys.data();
	}

	physx::PxDefaultMemoryOutputStream writeBuffer {};
	physx::PxConvexMeshCookingResult::Enum result;
	auto status = GetPxEnv().GetCooking().cookConvexMesh(meshDesc,writeBuffer,&result);
	if(status == false)
		return;

	physx::PxDefaultMemoryInputData readBuffer(writeBuffer.getData(),writeBuffer.getSize());
	auto *pConvexMesh = PxGetPhysics().createConvexMesh(readBuffer);

	m_bBuilt = true;

	m_geometry = std::unique_ptr<physx::PxGeometry,void(*)(physx::PxGeometry*)>(
		new physx::PxConvexMeshGeometry{pConvexMesh},
		[](physx::PxGeometry *p) {delete p;}
	);
	auto *surfMat = GetSurfaceMaterial();
	m_shape = px_create_unique_ptr(PxGetPhysics().createShape(*m_geometry,PxMaterial::GetMaterial(/*surfMat ? surfMat->GetPhysicsMaterial() : */GetPxEnv().GetGenericMaterial()).GetInternalObject(),false));
	GetPxEnv().InitializeShape(*this);
}
void pragma::physics::PxConvexHullShape::AddTriangle(uint32_t idx0,uint32_t idx1,uint32_t idx2)
{
	for(auto idx : {idx0,idx1,idx2})
		m_triangles.push_back(idx);
}
void pragma::physics::PxConvexHullShape::ReservePoints(uint32_t numPoints)
{
	m_vertices.reserve(numPoints);
}
void pragma::physics::PxConvexHullShape::ReserveTriangles(uint32_t numTris)
{
	m_triangles.reserve(numTris *3);
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

//////////////

pragma::physics::PxCompoundShape::PxCompoundShape(IEnvironment &env)
	: IShape{env},ICompoundShape{env},PxShape{env,px_null_ptr<physx::PxShape>(),px_null_ptr<physx::PxGeometry>()}
{}
void pragma::physics::PxCompoundShape::AddShape(pragma::physics::IShape &shape)
{
	m_shapes.push_back(std::static_pointer_cast<IShape>(shape.shared_from_this()));
}

//////////////

pragma::physics::PxTriangleShape::PxTriangleShape(IEnvironment &env)
	: ITriangleShape{env},PxShape{env,px_null_ptr<physx::PxShape>(),px_null_ptr<physx::PxGeometry>()},IShape{env}
{}
void pragma::physics::PxTriangleShape::CalculateLocalInertia(float,Vector3 *localInertia) const
{
	*localInertia = Vector3(0.f,0.f,0.f);
}
void pragma::physics::PxTriangleShape::Build(const std::vector<SurfaceMaterial> *materials)
{
	m_shape = nullptr;
	auto &verts = GetVertices();
	static_assert(sizeof(decltype(verts.front())) == sizeof(physx::PxVec3),"Vertex size mismatch!");
	physx::PxTriangleMeshDesc meshDesc;
	meshDesc.points.count = verts.size(); // TODO: Convert to PhysX coordinate system
	meshDesc.points.stride = sizeof(verts.front());
	meshDesc.points.data = verts.data();
	meshDesc.flags = physx::PxMeshFlag::e16_BIT_INDICES;

	auto &tris = GetTriangles();
	static_assert(std::is_same_v<std::remove_reference_t<decltype(tris.front())>,physx::PxU32>,"Vertex index type mismatch!");
	meshDesc.triangles.count = tris.size() /3;
	meshDesc.triangles.stride = 3 *sizeof(tris.front());
	meshDesc.triangles.data = tris.data();

	std::vector<physx::PxMaterialTableIndex> pxIndices {};
	std::vector<physx::PxMaterial*> pxMaterials {};
	if(materials)
	{
		auto &matIndices = GetSurfaceMaterials();
		
		pxIndices.reserve(matIndices.size());
		std::unordered_map<PxMaterial*,size_t> materialIndices {};
		for(auto idx : matIndices)
		{
			auto &mat = materials->at(idx);
			auto &physMat = PxMaterial::GetMaterial(mat.GetPhysicsMaterial());
			auto it = materialIndices.find(&physMat);
			if(it == materialIndices.end())
			{
				it = materialIndices.insert(std::make_pair(&physMat,pxMaterials.size())).first;
				pxMaterials.push_back(&physMat.GetInternalObject());
			}
			pxIndices.push_back(it->second);
		}
		meshDesc.materialIndices.data = pxIndices.data();
		meshDesc.materialIndices.stride = sizeof(physx::PxMaterialTableIndex);
	}

	physx::PxDefaultMemoryOutputStream writeBuffer {};
	physx::PxTriangleMeshCookingResult::Enum result;
	auto status = GetPxEnv().GetCooking().cookTriangleMesh(meshDesc,writeBuffer,&result);
	if(status == false)
		return;
	
	physx::PxDefaultMemoryInputData readBuffer(writeBuffer.getData(),writeBuffer.getSize());
	auto *pTriMesh = PxGetPhysics().createTriangleMesh(readBuffer);

	m_bBuilt = true;

	m_geometry = std::unique_ptr<physx::PxGeometry,void(*)(physx::PxGeometry*)>(
		new physx::PxTriangleMeshGeometry{pTriMesh},
		[](physx::PxGeometry *p) {delete p;}
	);
	auto *surfMat = GetSurfaceMaterial();
	m_shape = px_create_unique_ptr(PxGetPhysics().createShape(*m_geometry,PxMaterial::GetMaterial(/*surfMat ? surfMat->GetPhysicsMaterial() : */GetPxEnv().GetGenericMaterial()).GetInternalObject(),false));
	GetPxEnv().InitializeShape(*this);
	m_shape->setMaterials(pxMaterials.data(),pxMaterials.size());
}
#pragma optimize("",on)
