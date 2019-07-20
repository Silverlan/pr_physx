#ifndef __PR_PX_SHAPE_HPP__
#define __PR_PX_SHAPE_HPP__

#include <pragma/physics/shape.hpp>
#include <pragma/physics/transform.hpp>
#include "pr_physx/common.hpp"

namespace physx
{
	class PxShape;
	class PxGeometry;
	class PxGeometryHolder;
};
namespace pragma::physics
{
	class IEnvironment;
	class PhysXEnvironment;
	class PhysXShape
		: virtual public pragma::physics::IShape
	{
	public:
		friend PhysXEnvironment;
		friend IEnvironment;

		static PhysXShape &GetShape(IShape &s);
		static const PhysXShape &GetShape(const IShape &s);

		const physx::PxGeometryHolder &GetInternalObject() const;
		physx::PxGeometryHolder &GetInternalObject();

		virtual void SetLocalPose(const physics::Transform &localPose) override;
		virtual physics::Transform GetLocalPose() const override;

		virtual void CalculateLocalInertia(float mass,Vector3 *localInertia) const override;
		virtual void GetAABB(Vector3 &min,Vector3 &max) const override;
		virtual void GetBoundingSphere(Vector3 &outCenter,float &outRadius) const override;

		virtual void ApplySurfaceMaterial(IMaterial &mat) override;
		IMaterial *GetMaterial() const;

		virtual void SetMass(float mass) override;
		virtual float GetMass() const override;

		virtual bool IsValid() const override;
		PhysXEnvironment &GetPxEnv() const;
	protected:
		PhysXShape(IEnvironment &env,const std::shared_ptr<physx::PxGeometry> &geometry);
		void UpdateBounds();
		std::shared_ptr<physx::PxGeometry> m_geometry = nullptr;
		physx::PxGeometryHolder m_geometryHolder = {};
		std::shared_ptr<IMaterial> m_material = nullptr;
		std::pair<Vector3,Vector3> m_bounds = {};
		physics::Transform m_localPose = {};
		float m_mass = 0.f;
	};

	class PhysXConvexShape
		: virtual public pragma::physics::IConvexShape,
		public PhysXShape
	{
	public:
		friend PhysXEnvironment;
		friend IEnvironment;

		virtual void SetLocalScaling(const Vector3 &scale) override;
	protected:
		PhysXConvexShape(IEnvironment &env,const std::shared_ptr<physx::PxGeometry> &geometry);
	};

	class PhysXConvexHullShape
		: virtual public pragma::physics::IConvexHullShape,
		public PhysXConvexShape
	{
	public:
		friend IEnvironment;

		virtual void AddPoint(const Vector3 &point) override;

		virtual void AddTriangle(uint32_t idx0,uint32_t idx1,uint32_t idx2) override;
		virtual void ReservePoints(uint32_t numPoints) override;
		virtual void ReserveTriangles(uint32_t numTris) override;
		virtual void DoBuild() override;
	private:
		PhysXConvexHullShape(IEnvironment &env);
		std::vector<Vector3> m_vertices = {};
		std::vector<uint16_t> m_triangles = {};
	};

	class PhysXCapsuleShape
		: virtual public pragma::physics::ICapsuleShape,
		public PhysXConvexShape
	{
	public:
		friend PhysXEnvironment;
		friend IEnvironment;

		virtual float GetRadius() const override;
		virtual float GetHalfHeight() const override;
	protected:
		PhysXCapsuleShape(IEnvironment &env,const std::shared_ptr<physx::PxGeometry> &geometry);
	};

	class PhysXBoxShape
		: virtual public pragma::physics::IBoxShape,
		public PhysXConvexShape
	{
	public:
		friend PhysXEnvironment;
		friend IEnvironment;
		virtual Vector3 GetHalfExtents() const override;
	protected:
		PhysXBoxShape(IEnvironment &env,const std::shared_ptr<physx::PxGeometry> &geometry);
	};

	class PhysXCompoundShape
		: virtual public pragma::physics::ICompoundShape,
		public PhysXShape
	{
	public:
		friend IEnvironment;
		virtual bool IsValid() const override;
		virtual void SetMass(float mass) override;
		virtual float GetMass() const override;
	protected:
		PhysXCompoundShape(IEnvironment &env);

	};

	class IMaterial;
	class PhysXTriangleShape
		: virtual public ITriangleShape,
		public PhysXShape
	{
	public:
		friend PhysXEnvironment;
		friend IEnvironment;
		virtual void CalculateLocalInertia(float mass,Vector3 *localInertia) const override;
		//virtual void AddTriangle(const Vector3 &a,const Vector3 &b,const Vector3 &c,const SurfaceMaterial *mat=nullptr);
		virtual void DoBuild(const std::vector<SurfaceMaterial> *materials=nullptr) override;
	protected:
		PhysXTriangleShape(IEnvironment &env);
	};

	///////////////

	class PhysXActorShapeCollection;
	class PhysXActorShape
		: public IBase
	{
	public:
		friend PhysXActorShapeCollection;

		void SetTrigger(bool bTrigger);
		bool IsTrigger() const;

		void ApplySurfaceMaterial(IMaterial &mat);

		void SetLocalPose(const Transform &t);
		Transform GetLocalPose() const;

		PhysXShape &GetShape() const;
		physx::PxShape &GetActorShape() const;

		PhysXEnvironment &GetPxEnv() const;
	private:
		using IBase::shared_from_this;
		PhysXActorShape(IEnvironment &env,physx::PxShape &actorShape,PhysXShape &shape);

		physx::PxShape &m_actorShape;
		std::shared_ptr<PhysXShape> m_shape = nullptr;
	};
};

#endif
