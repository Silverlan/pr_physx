# pr_physx
PhysX physics module for the Pragma game engine.

Building PhysX:
- Open third_party/physx/physx/buildtools/presets/public
- Set "NV_USE_STATIC_WINCRT" to "False"
- Run Visual Studio Command Prompt (Latest version may not work)
- Add Python to PATH
- Run msys2 (msys2_shell.cmd -use-full-path)
- Execute generate_projects.bat
- Use 64-Bit configuration for latest Visual Studio version
- Open generated solution and build all PhysX libraries in release mode
- Move the following DLLs to "install/modules/physics_engines/physx":
	- PhysX_64.dll
	- PhysXCommon_64.dll
	- PhysXCooking_64.dll
	- PhysXFoundation_64.dll