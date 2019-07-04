#ifndef __PR_PX_SHAPE_HPP__
#define __PR_PX_SHAPE_HPP__

#include <pragma/physics/shape.hpp>
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
};

#endif
