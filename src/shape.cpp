/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "pr_physx/shape.hpp"
#include "pr_physx/environment.hpp"
#include "pr_physx/material.hpp"
#include "pr_physx/vehicle.hpp"
#include "pr_physx/collision_object.hpp"
#include "pr_physx/common.hpp"
#include "pr_physx/query_filter_callback.hpp"
#include <pragma/math/surfacematerial.h>
#include <PxShape.h>
#include <common/PxCoreUtilityTypes.h>
#include <cooking/PxCooking.h>

pragma::physics::PhysXShape &pragma::physics::PhysXShape::GetShape(IShape &s) { return *static_cast<PhysXShape *>(s.GetUserData()); }
const pragma::physics::PhysXShape &pragma::physics::PhysXShape::GetShape(const IShape &o) { return GetShape(const_cast<IShape &>(o)); }
pragma::physics::PhysXShape::PhysXShape(IEnvironment &env, const std::shared_ptr<const physx::PxGeometry> &geometry) : IShape {env}, m_geometry {geometry}
{
	SetUserData(this);
	if(geometry) {
		m_geometryHolder = {*geometry};
		UpdateBounds();
	}
}
const physx::PxGeometryHolder &pragma::physics::PhysXShape::GetInternalObject() const { return m_geometryHolder; }
physx::PxGeometryHolder &pragma::physics::PhysXShape::GetInternalObject() { return m_geometryHolder; }
void pragma::physics::PhysXShape::SetLocalPose(const umath::Transform &localPose) { m_localPose = localPose; }
umath::Transform pragma::physics::PhysXShape::GetLocalPose() const { return m_localPose; }
void pragma::physics::PhysXShape::CalculateLocalInertia(float mass, Vector3 *localInertia) const
{
	// TODO
}
void pragma::physics::PhysXShape::GetAABB(Vector3 &min, Vector3 &max) const
{
	min = m_localPose * m_bounds.first;
	max = m_localPose * m_bounds.second;
}
void pragma::physics::PhysXShape::UpdateBounds()
{
	Vector3 min, max;
	auto type = m_geometry->getType();
	switch(type) {
	case physx::PxGeometryType::eBOX:
		{
			auto &box = m_geometryHolder.box();
			auto extents = GetPxEnv().FromPhysXVector(box.halfExtents);
			min = -extents;
			max = extents;
			break;
		}
	case physx::PxGeometryType::eSPHERE:
		{
			auto &sphere = m_geometryHolder.sphere();
			auto extents = Vector3 {sphere.radius, sphere.radius, sphere.radius};
			min = -extents;
			max = extents;
			break;
		}
	case physx::PxGeometryType::eCAPSULE:
		{
			auto &capsule = m_geometryHolder.capsule();
			auto extents = Vector3 {capsule.radius, capsule.halfHeight, capsule.radius};
			min = -extents;
			max = extents;
			break;
		}
	case physx::PxGeometryType::eCONVEXMESH:
	case physx::PxGeometryType::eTRIANGLEMESH:
		{
			const physx::PxVec3 *verts;
			physx::PxU32 numVerts;
			if(type == physx::PxGeometryType::eCONVEXMESH) {
				auto &convexMesh = m_geometryHolder.convexMesh();
				verts = convexMesh.convexMesh->getVertices();
				numVerts = convexMesh.convexMesh->getNbVertices();
			}
			else {
				auto &triMesh = m_geometryHolder.triangleMesh();
				verts = triMesh.triangleMesh->getVertices();
				numVerts = triMesh.triangleMesh->getNbVertices();
			}
			if(numVerts == 0) {
				min = {};
				max = {};
				break;
			}
			min = Vector3 {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
			max = Vector3 {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()};
			for(auto i = decltype(numVerts) {0u}; i < numVerts; ++i) {
				auto &v = reinterpret_cast<const Vector3 &>(verts[i]);
				static_assert(sizeof(v) == sizeof(Vector3));
				uvec::min(&min, v);
				uvec::max(&max, v);
			}
			break;
		}
	case physx::PxGeometryType::eHEIGHTFIELD:
		{
			// TODO
			Con::cwar << "Attempted to update AABB for heightfield, which is not yet implemented!" << Con::endl;
			min = {};
			max = {};
			break;
		}
	case physx::PxGeometryType::ePLANE:
		{
			// TODO
			Con::cwar << "Attempted to update AABB for plane, which is not yet implemented!" << Con::endl;
			min = {};
			max = {};
			break;
		}
	default:
		{
			// TODO
			Con::cwar << "Attempted to update AABB for unknown shape type!" << Con::endl;
			min = {};
			max = {};
			break;
		}
	}
	m_bounds = {min, max};
}
void pragma::physics::PhysXShape::GetBoundingSphere(Vector3 &outCenter, float &outRadius) const
{
	// TODO
}
void pragma::physics::PhysXShape::ApplySurfaceMaterial(IMaterial &mat) {}
pragma::physics::IMaterial *pragma::physics::PhysXShape::GetMaterial() const { return m_material.get(); }
void pragma::physics::PhysXShape::SetMass(float mass) { m_mass = mass; }
float pragma::physics::PhysXShape::GetMass() const { return m_mass; }
bool pragma::physics::PhysXShape::IsValid() const { return IShape::IsValid() && m_geometry != nullptr; }
pragma::physics::PhysXEnvironment &pragma::physics::PhysXShape::GetPxEnv() const { return static_cast<PhysXEnvironment &>(m_physEnv); }

//////////////

pragma::physics::PhysXConvexShape::PhysXConvexShape(IEnvironment &env, const std::shared_ptr<const physx::PxGeometry> &geometry) : IConvexShape {env}, PhysXShape {env, std::move(geometry)}, IShape {env} {}
void pragma::physics::PhysXConvexShape::SetLocalScaling(const Vector3 &scale)
{
	// TODO
}

//////////////

pragma::physics::PhysXConvexHullShape::PhysXConvexHullShape(IEnvironment &env) : IConvexHullShape {env}, IShape {env}, IConvexShape {env}, PhysXConvexShape {env, px_null_ptr<physx::PxGeometry>()} {}
void pragma::physics::PhysXConvexHullShape::AddPoint(const Vector3 &point) { m_vertices.push_back(point); }
void pragma::physics::PhysXConvexHullShape::DoBuild()
{
	// TODO: Convert to PhysX coordinate system
	static_assert(sizeof(decltype(m_vertices.front())) == sizeof(physx::PxVec3), "Vertex size mismatch!");
	physx::PxConvexMeshDesc meshDesc;
	meshDesc.points.count = m_vertices.size();
	meshDesc.points.stride = sizeof(m_vertices.front());
	meshDesc.points.data = m_vertices.data();

	std::vector<physx::PxHullPolygon> polys {};
	if(true) //m_triangles.empty()) // TODO
	{
		meshDesc.flags = physx::PxConvexFlag::eCOMPUTE_CONVEX;
		meshDesc.vertexLimit = 255;
	}
	else {
		meshDesc.flags = physx::PxConvexFlag::e16_BIT_INDICES;
		meshDesc.indices.data = m_triangles.data();
		meshDesc.indices.count = m_triangles.size();
		meshDesc.indices.stride = sizeof(m_triangles.front());

		polys.reserve(m_triangles.size() / 3);
		for(auto i = decltype(m_triangles.size()) {0u}; i < m_triangles.size(); i += 3) {
			polys.push_back({});
			auto &poly = polys.back();
			auto idx0 = m_triangles.at(i);
			auto idx1 = m_triangles.at(i + 1);
			auto idx2 = m_triangles.at(i + 2);

			Vector3 n;
			float d;
			uvec::calc_plane(m_vertices.at(idx0), m_vertices.at(idx1), m_vertices.at(idx2), n, d);

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
	physx::PxCookingParams params {physx::PxTolerancesScale {}};
	auto status = PxCookConvexMesh(params, meshDesc, writeBuffer, &result);
	if(status == false)
		return;

	physx::PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
	auto *pConvexMesh = PxGetPhysics().createConvexMesh(readBuffer);

	m_bBuilt = true;

	m_geometry = std::unique_ptr<physx::PxGeometry, void (*)(physx::PxGeometry *)>(new physx::PxConvexMeshGeometry {pConvexMesh}, [](physx::PxGeometry *p) { delete p; });
	m_geometryHolder = {*m_geometry};
	UpdateBounds();
}
void pragma::physics::PhysXConvexHullShape::AddTriangle(uint32_t idx0, uint32_t idx1, uint32_t idx2)
{
	for(auto idx : {idx0, idx1, idx2})
		m_triangles.push_back(idx);
}
void pragma::physics::PhysXConvexHullShape::ReservePoints(uint32_t numPoints) { m_vertices.reserve(numPoints); }
void pragma::physics::PhysXConvexHullShape::ReserveTriangles(uint32_t numTris) { m_triangles.reserve(numTris * 3); }

//////////////

pragma::physics::PhysXCapsuleShape::PhysXCapsuleShape(IEnvironment &env, const std::shared_ptr<const physx::PxGeometry> &geometry) : ICapsuleShape {env}, PhysXConvexShape {env, std::move(geometry)}, IShape {env}, IConvexShape {env} {}

float pragma::physics::PhysXCapsuleShape::GetRadius() const { return GetPxEnv().FromPhysXLength(GetInternalObject().capsule().radius); }
float pragma::physics::PhysXCapsuleShape::GetHalfHeight() const { return GetPxEnv().FromPhysXLength(GetInternalObject().capsule().halfHeight * 0.5f); }

//////////////

pragma::physics::PhysXBoxShape::PhysXBoxShape(IEnvironment &env, const std::shared_ptr<const physx::PxGeometry> &geometry) : IBoxShape {env}, PhysXConvexShape {env, std::move(geometry)}, IShape {env}, IConvexShape {env} {}

Vector3 pragma::physics::PhysXBoxShape::GetHalfExtents() const { return GetPxEnv().FromPhysXVector(GetInternalObject().box().halfExtents); }

//////////////

pragma::physics::PhysXCompoundShape::PhysXCompoundShape(IEnvironment &env) : IShape {env}, ICompoundShape {env}, PhysXShape {env, nullptr} {}
bool pragma::physics::PhysXCompoundShape::IsValid() const
{
	auto &shapes = GetShapes();
	return std::find_if(shapes.begin(), shapes.end(), [](const pragma::physics::ICompoundShape::ShapeInfo &shapeInfo) { return shapeInfo.shape->IsValid() == false; }) == shapes.end();
}
void pragma::physics::PhysXCompoundShape::SetMass(float mass) { ICompoundShape::SetMass(mass); }
float pragma::physics::PhysXCompoundShape::GetMass() const { return ICompoundShape::GetMass(); }
void pragma::physics::PhysXCompoundShape::GetAABB(Vector3 &min, Vector3 &max) const { return ICompoundShape::GetAABB(min, max); }

//////////////

pragma::physics::PhysXTriangleShape::PhysXTriangleShape(IEnvironment &env) : ITriangleShape {env}, PhysXShape {env, nullptr}, IShape {env} {}
void pragma::physics::PhysXTriangleShape::CalculateLocalInertia(float, Vector3 *localInertia) const { *localInertia = Vector3(0.f, 0.f, 0.f); }
void pragma::physics::PhysXTriangleShape::DoBuild(const std::vector<SurfaceMaterial> *materials)
{
	m_geometry = nullptr;
	auto &verts = GetVertices();
	static_assert(sizeof(decltype(verts.front())) == sizeof(physx::PxVec3), "Vertex size mismatch!");
	physx::PxTriangleMeshDesc meshDesc;
	meshDesc.points.count = verts.size(); // TODO: Convert to PhysX coordinate system
	meshDesc.points.stride = sizeof(verts.front());
	meshDesc.points.data = verts.data();
	meshDesc.flags = static_cast<physx::PxMeshFlags>(0);

	auto &tris = GetTriangles();
	static_assert(std::is_same_v<std::remove_reference_t<decltype(tris.front())>, physx::PxU32>, "Vertex index type mismatch!");
	meshDesc.triangles.count = tris.size() / 3;
	meshDesc.triangles.stride = 3 * sizeof(tris.front());
	meshDesc.triangles.data = tris.data();

	std::vector<physx::PxMaterialTableIndex> pxIndices {};
	std::vector<physx::PxMaterial *> pxMaterials {};
	if(materials) {
		auto &matIndices = GetSurfaceMaterials();

		pxIndices.reserve(matIndices.size());
		std::unordered_map<PhysXMaterial *, size_t> materialIndices {};
		for(auto idx : matIndices) {
			auto &mat = materials->at(idx);
			auto &physMat = PhysXMaterial::GetMaterial(mat.GetPhysicsMaterial());
			auto it = materialIndices.find(&physMat);
			if(it == materialIndices.end()) {
				it = materialIndices.insert(std::make_pair(&physMat, pxMaterials.size())).first;
				pxMaterials.push_back(&physMat.GetInternalObject());
			}
			pxIndices.push_back(it->second);
		}
		meshDesc.materialIndices.data = pxIndices.data();
		meshDesc.materialIndices.stride = sizeof(physx::PxMaterialTableIndex);
	}

	physx::PxDefaultMemoryOutputStream writeBuffer {};
	physx::PxTriangleMeshCookingResult::Enum result;
	physx::PxCookingParams params {physx::PxTolerancesScale {}};
	auto status = PxCookTriangleMesh(params, meshDesc, writeBuffer, &result);
	if(status == false)
		return;

	physx::PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
	auto *pTriMesh = PxGetPhysics().createTriangleMesh(readBuffer);

	m_bBuilt = true;

	m_geometry = std::unique_ptr<physx::PxGeometry, void (*)(physx::PxGeometry *)>(new physx::PxTriangleMeshGeometry {pTriMesh}, [](physx::PxGeometry *p) { delete p; });
	m_geometryHolder = {*m_geometry};
	UpdateBounds();
}

///////////////

pragma::physics::PhysXActorShape::PhysXActorShape(IEnvironment &env, physx::PxShape &actorShape, PhysXShape &shape) : IBase {env}, m_shape {std::dynamic_pointer_cast<PhysXShape>(shape.shared_from_this())}, m_actorShape {actorShape} {}
pragma::physics::PhysXEnvironment &pragma::physics::PhysXActorShape::GetPxEnv() const { return static_cast<PhysXEnvironment &>(m_physEnv); }
void pragma::physics::PhysXActorShape::SetTrigger(bool bTrigger)
{
	m_actorShape.setFlag(physx::PxShapeFlag::eSIMULATION_SHAPE, !bTrigger);
	m_actorShape.setFlag(physx::PxShapeFlag::eTRIGGER_SHAPE, bTrigger);
}
bool pragma::physics::PhysXActorShape::IsTrigger() const { return m_actorShape.getFlags().isSet(physx::PxShapeFlag::eTRIGGER_SHAPE); }

void pragma::physics::PhysXActorShape::ApplySurfaceMaterial(IMaterial &mat)
{
	auto numMats = m_actorShape.getNbMaterials();
	std::vector<physx::PxMaterial *> materials {numMats};
	numMats = m_actorShape.getMaterials(materials.data(), materials.size());
	materials.resize(numMats);
	if(materials.empty())
		return;
	materials.front() = &PhysXMaterial::GetMaterial(mat).GetInternalObject();
	m_actorShape.setMaterials(materials.data(), materials.size());
}

void pragma::physics::PhysXActorShape::SetLocalPose(const umath::Transform &t) { m_actorShape.setLocalPose(GetPxEnv().CreatePxTransform(t)); }
umath::Transform pragma::physics::PhysXActorShape::GetLocalPose() const { return GetPxEnv().CreateTransform(m_actorShape.getLocalPose()); }
pragma::physics::PhysXShape &pragma::physics::PhysXActorShape::GetShape() const { return *m_shape; }
physx::PxShape &pragma::physics::PhysXActorShape::GetActorShape() const { return m_actorShape; }

///////////

pragma::physics::PhysXActorShapeCollection::PhysXActorShapeCollection(PhysXCollisionObject &colObj) : m_collisionObject {colObj} {}
pragma::physics::PhysXActorShape *pragma::physics::PhysXActorShapeCollection::AttachShapeToActor(PhysXShape &shape, PhysXMaterial &mat, const umath::Transform &localPose)
{
	if(m_collisionObject.IsRigid() == false)
		return nullptr;
	auto *pxActorShape = physx::PxRigidActorExt::createExclusiveShape(static_cast<PhysXRigidBody &>(m_collisionObject).GetInternalObject(), shape.GetInternalObject().any(), mat.GetInternalObject());
	auto pose = localPose * shape.GetLocalPose();
	pxActorShape->setLocalPose(shape.GetPxEnv().CreatePxTransform(pose));
	return AddShape(shape, *pxActorShape, true);
}
pragma::physics::PhysXActorShape *pragma::physics::PhysXActorShapeCollection::AddShape(PhysXShape &shape, physx::PxShape &pxActorShape) { return AddShape(shape, pxActorShape, false); }
pragma::physics::PhysXActorShape *pragma::physics::PhysXActorShapeCollection::AddShape(PhysXShape &shape, physx::PxShape &pxActorShape, bool applyPose)
{
	auto actorShape = std::unique_ptr<PhysXActorShape> {new PhysXActorShape {shape.GetPxEnv(), pxActorShape, shape}};
	shape.GetPxEnv().InitializeShape(*actorShape, applyPose);

	m_actorShapes.push_back(std::move(actorShape));
	return m_actorShapes.back().get();
}
const std::vector<std::unique_ptr<pragma::physics::PhysXActorShape>> &pragma::physics::PhysXActorShapeCollection::GetActorShapes() const { return m_actorShapes; }

void pragma::physics::PhysXActorShapeCollection::SetTrigger(bool bTrigger)
{
	for(auto &actorShape : m_actorShapes)
		actorShape->SetTrigger(bTrigger);
}
bool pragma::physics::PhysXActorShapeCollection::IsTrigger() const
{
	if(m_actorShapes.empty())
		return false;
	return m_actorShapes.front()->IsTrigger();
}

void pragma::physics::PhysXActorShapeCollection::ApplySurfaceMaterial(PhysXMaterial &mat)
{
	for(auto &actorShape : m_actorShapes)
		actorShape->ApplySurfaceMaterial(mat);
}

void pragma::physics::PhysXActorShapeCollection::TransformLocalPose(const umath::Transform &t)
{
	for(auto &actorShape : m_actorShapes)
		actorShape->SetLocalPose(t * actorShape->GetLocalPose());
}
void pragma::physics::PhysXActorShapeCollection::Clear() { m_actorShapes.clear(); }
void pragma::physics::PhysXActorShapeCollection::CalcMassProps(float mass, Vector3 &centerOfMass)
{
	if(m_actorShapes.empty()) {
		mass = 0.f;
		centerOfMass = {};
		return;
	}
	std::vector<physx::PxShape *> shapes {};
	shapes.reserve(m_actorShapes.size());
	for(auto &actorShape : m_actorShapes)
		shapes.push_back(&actorShape->GetActorShape());
	auto massProps = physx::PxRigidBodyExt::computeMassPropertiesFromShapes(shapes.data(), shapes.size());
	mass = m_actorShapes.front()->GetPxEnv().FromPhysXMass(massProps.mass);
	centerOfMass = m_actorShapes.front()->GetPxEnv().FromPhysXVector(massProps.centerOfMass);
}
