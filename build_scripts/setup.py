import os
from sys import platform
import subprocess

module_root = os.path.dirname(os.path.realpath(__file__)) +"/../"
physx_root = module_root +"third_party/physx/physx/"
os.chdir(physx_root)
if platform == "linux":
	subprocess.run([physx_root +"generate_projects.sh"],check=True,shell=True)
else:
	subprocess.run([physx_root +"generate_projects.bat","vc17win64"],check=True,shell=True)

physx_lib_dir = physx_root +"bin/win.x86_64.vc143.mt/release/"
cmake_args.append("-DDEPENDENCY_PHYSX_INCLUDE=" +physx_root +"include/")
cmake_args.append("-DDEPENDENCY_PHYSX_LIBRARY=" +physx_lib_dir +"PhysX_64.lib")
cmake_args.append("-DDEPENDENCY_PHYSX_EXTENSIONS_LIBRARY=" +physx_lib_dir +"PhysXExtensions_static_64.lib")
cmake_args.append("-DDEPENDENCY_PHYSX_FOUNDATION_LIBRARY=" +physx_lib_dir +"PhysXFoundation_64.lib")
cmake_args.append("-DDEPENDENCY_PHYSX_VEHICLE_LIBRARY=" +physx_lib_dir +"PhysXVehicle_static_64.lib")
cmake_args.append("-DDEPENDENCY_PHYSX_COMMON_LIBRARY=" +physx_lib_dir +"PhysXCommon_64.lib")
cmake_args.append("-DDEPENDENCY_PHYSX_CHARACTER_KINEMATIC_LIBRARY=" +physx_lib_dir +"PhysXCharacterKinematic_static_64.lib")
cmake_args.append("-DDEPENDENCY_PHYSX_COOKING_LIBRARY=" +physx_lib_dir +"PhysXCooking_64.lib")
cmake_args.append("-DDEPENDENCY_PHYSX_PVD_LIBRARY=" +physx_lib_dir +"PhysXPvdSDK_static_64.lib")
