#ifndef __PR_PX_SHAPE_HPP__
#define __PR_PX_SHAPE_HPP__

#include <pragma/physics/shape.hpp>
#include <pragma/physics/transform.hpp>
#include "pr_physx/common.hpp"

namespace physx
{
	class PxShape;
	class PxGeometry;
};
namespace pragma::physics
{
	class IEnvironment;
	class PxEnvironment;
	class PxShape
		: virtual public pragma::physics::IShape
	{
	public:
		friend PxEnvironment;
		friend IEnvironment;
		static PxShape &GetShape(IShape &s);
		static const PxShape &GetShape(const IShape &s);

		const physx::PxShape &GetInternalObject() const;
		physx::PxShape &GetInternalObject();

		virtual void CalculateLocalInertia(float mass,Vector3 *localInertia) const override;
		virtual void GetAABB(Vector3 &min,Vector3 &max) const override;
		virtual void GetBoundingSphere(Vector3 &outCenter,float &outRadius) const override;

		virtual void SetTrigger(bool bTrigger) override;
		virtual bool IsTrigger() const override;

		virtual void SetLocalPose(const Transform &t) override;
		virtual Transform GetLocalPose() const override;

		virtual bool IsValid() const override;

		PxEnvironment &GetPxEnv() const;
	protected:
		PxShape(IEnvironment &env,PxUniquePtr<physx::PxShape> shape,PxUniquePtr<physx::PxGeometry> geometry);
		PxUniquePtr<physx::PxGeometry> m_geometry = px_null_ptr<physx::PxGeometry>();
		PxUniquePtr<physx::PxShape> m_shape = px_null_ptr<physx::PxShape>();
	};

	class PxConvexShape
		: virtual public pragma::physics::IConvexShape,
		public PxShape
	{
	public:
		friend PxEnvironment;
		friend IEnvironment;

		virtual void SetLocalScaling(const Vector3 &scale) override;
	protected:
		PxConvexShape(IEnvironment &env,PxUniquePtr<physx::PxShape> shape,PxUniquePtr<physx::PxGeometry> geometry);
	};

	class PxConvexHullShape
		: virtual public pragma::physics::IConvexHullShape,
		public PxConvexShape
	{
	public:
		friend IEnvironment;

		virtual void AddPoint(const Vector3 &point) override;

		virtual void AddTriangle(uint32_t idx0,uint32_t idx1,uint32_t idx2) override;
		virtual void ReservePoints(uint32_t numPoints) override;
		virtual void ReserveTriangles(uint32_t numTris) override;
		virtual void Build() override;
	private:
		PxConvexHullShape(IEnvironment &env);
		std::vector<Vector3> m_vertices = {};
		std::vector<uint16_t> m_triangles = {};
	};

	class PxCapsuleShape
		: virtual public pragma::physics::ICapsuleShape,
		public PxConvexShape
	{
	public:
		friend PxEnvironment;
		friend IEnvironment;

		virtual float GetRadius() const override;
		virtual float GetHalfHeight() const override;
	protected:
		PxCapsuleShape(IEnvironment &env,PxUniquePtr<physx::PxShape> shape,PxUniquePtr<physx::PxGeometry> geometry);
	};

	class PxBoxShape
		: virtual public pragma::physics::IBoxShape,
		public PxConvexShape
	{
	public:
		friend PxEnvironment;
		friend IEnvironment;
		virtual Vector3 GetHalfExtents() const override;
	protected:
		PxBoxShape(IEnvironment &env,PxUniquePtr<physx::PxShape> shape,PxUniquePtr<physx::PxGeometry> geometry);
	};

	class PxCompoundShape
		: virtual public pragma::physics::ICompoundShape,
		public PxShape
	{
	public:
		friend IEnvironment;
		virtual void AddShape(pragma::physics::IShape &shape) override;
	protected:
		PxCompoundShape(IEnvironment &env);
	};

	class IMaterial;
	class PxTriangleShape
		: virtual public ITriangleShape,
		public PxShape
	{
	public:
		friend PxEnvironment;
		friend IEnvironment;
		virtual void CalculateLocalInertia(float mass,Vector3 *localInertia) const override;
		//virtual void AddTriangle(const Vector3 &a,const Vector3 &b,const Vector3 &c,const SurfaceMaterial *mat=nullptr);
		virtual void Build(const std::vector<SurfaceMaterial> *materials=nullptr) override;
	protected:
		PxTriangleShape(IEnvironment &env);
	};
};

#endif
