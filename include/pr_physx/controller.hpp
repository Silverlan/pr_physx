#ifndef __PR_PX_CONTROLLER_HPP__
#define __PR_PX_CONTROLLER_HPP__

#include "common.hpp"
#include <pragma/physics/controller.hpp>
#include <mathutil/uvec.h>

namespace physx
{
	class PxController;
};
namespace pragma::physics
{
	class PxEnvironment;
	class PxController
		: virtual public IController
	{
	public:
		friend IEnvironment;
		physx::PxController &GetInternalObject() const;

		virtual CollisionFlags DoMove(Vector3 &disp) override;
		virtual Vector3 GetDimensions() const override;
		virtual void SetDimensions(const Vector3 &dimensions) override;
		virtual void Resize(float newHeight) override;
	protected:
		PxController(IEnvironment &env,PxUniquePtr<physx::PxController> controller,const util::TSharedHandle<ICollisionObject> &collisionObject);
		PxEnvironment &GetPxEnv() const;
		PxUniquePtr<physx::PxController> m_controller = px_null_ptr<physx::PxController>();
	};
};

#endif
