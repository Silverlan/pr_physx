include(${CMAKE_SOURCE_DIR}/cmake/pr_common.cmake)

set(PROJ_NAME pr_physx)
pr_add_library(${PROJ_NAME} SHARED)

pr_add_dependency(${PROJ_NAME} shared TARGET PUBLIC)

pr_set_include_path(physx "${CMAKE_CURRENT_LIST_DIR}/third_party/physx/physx/include" FORCE)
pr_set_include_path(physx_shared "${CMAKE_CURRENT_LIST_DIR}/third_party/physx/pxshared/include" FORCE)

pr_add_external_dependency(${PROJ_NAME} physx LIBRARY)
pr_add_external_dependency(${PROJ_NAME} physx_extensions LIBRARY LINK_ONLY)
pr_add_external_dependency(${PROJ_NAME} physx_foundation LIBRARY LINK_ONLY)
pr_add_external_dependency(${PROJ_NAME} physx_vehicle LIBRARY LINK_ONLY)
pr_add_external_dependency(${PROJ_NAME} physx_common LIBRARY LINK_ONLY)
pr_add_external_dependency(${PROJ_NAME} physx_character_kinematic LIBRARY LINK_ONLY)
pr_add_external_dependency(${PROJ_NAME} physx_cooking LIBRARY LINK_ONLY)
pr_add_external_dependency(${PROJ_NAME} physx_pvd LIBRARY LINK_ONLY)
pr_add_external_dependency(${PROJ_NAME} physx_shared HEADER_ONLY)

pr_add_headers(${PROJ_NAME} "include/")
pr_add_sources(${PROJ_NAME} "src/")

pr_finalize(${PROJ_NAME} FOLDER "modules/physics/physx")
