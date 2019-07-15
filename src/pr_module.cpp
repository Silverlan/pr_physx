#include "pr_module.hpp"
#include "pr_physx/environment.hpp"
#include <sharedutils/util_weak_handle.hpp>
#include <mathutil/umath.h>
#include <iostream>
#include <array>

extern "C"
{
	PRAGMA_EXPORT void initialize_physics_engine(NetworkState &nw,std::unique_ptr<pragma::physics::IEnvironment,void(*)(pragma::physics::IEnvironment*)> &outEnv)
	{
		auto env = std::unique_ptr<pragma::physics::IEnvironment,void(*)(pragma::physics::IEnvironment*)>{
			new pragma::physics::PhysXEnvironment{nw},[](pragma::physics::IEnvironment *env) {
			env->OnRemove();
			delete env;
		}};
		if(env->Initialize() == false)
			env = nullptr;
		outEnv = std::move(env);
	}
};
