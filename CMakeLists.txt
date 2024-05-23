cmake_minimum_required(VERSION 3.28)

include("../CMakeShared.txt")

set(PROJ_NAME pr_physx)

project(${PROJ_NAME} CXX)

# set(PRECOMPILED_HEADER "stdafx")

link_external_library(physx)
add_external_library(physx_extensions)
add_external_library(physx_foundation)
add_external_library(physx_vehicle)
add_external_library(physx_common)
add_external_library(physx_character_kinematic)
add_external_library(physx_cooking)
add_external_library(physx_pvd)
add_include_dir(physx_shared)
add_include_dir(glm)

set(DEPENDENCY_PHYSX_INCLUDE ${CMAKE_CURRENT_LIST_DIR}/third_party/physx/physx/include CACHE PATH "" FORCE)
set(DEPENDENCY_PHYSX_SHARED_INCLUDE ${CMAKE_CURRENT_LIST_DIR}/third_party/physx/pxshared/include CACHE PATH "" FORCE)

link_generic_module_libraries()
register_pragma_module(${PROJ_NAME})

#

set_target_properties(pr_physx PROPERTIES FOLDER modules/physics/physx)

set_property(GLOBAL PROPERTY PRAGMA_MODULE_SKIP_TARGET_PROPERTY_FOLDER 1)