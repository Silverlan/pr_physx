#ifndef __PR_PX_COMMON_HPP__
#define __PR_PX_COMMON_HPP__

#include <PxPhysicsAPI.h>
#include <mathutil/uvec.h>

namespace pragma::physics
{
	template<class T>
		using PxUniquePtr = std::unique_ptr<T,void(*)(T*)>;
	template<class T>
		static constexpr PxUniquePtr<T> px_null_ptr() {return PxUniquePtr<T>{nullptr,[](T*) {}};}
	template<class T>
		static PxUniquePtr<T> px_create_unique_ptr(T *v) {
			if(v == nullptr)
				return px_null_ptr<T>();
			return PxUniquePtr<T>{v,[](T *v) {if(v) v->release();}};
		}
	template<class TSrc,class TDst>
		static PxUniquePtr<TDst> px_cast_unique_ptr(PxUniquePtr<TSrc> ptr) {
			return px_create_unique_ptr<TDst>(ptr.release());
		}
};

namespace uvec
{
	physx::PxVec3 create_px(const Vector3 &v);
	Vector3 create(const physx::PxVec3 &v);
};

namespace uquat
{
	physx::PxQuat create_px(const Quat &v);
	Quat create(const physx::PxQuat &v);
};

#endif
