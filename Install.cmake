set(INSTALL_PATH "modules/physics_engines/physx/")
pr_install_create_directory("${INSTALL_PATH}")
pr_install_targets(pr_physx INSTALL_DIR "${INSTALL_PATH}")
pr_install_files(
    "${DEPENDENCY_PHYSX_LIBRARY}"
    "${DEPENDENCY_PHYSX_COMMON_LIBRARY}"
    "${DEPENDENCY_PHYSX_COOKING_LIBRARY}"
    "${DEPENDENCY_PHYSX_FOUNDATION_LIBRARY}"
    INSTALL_DIR "${INSTALL_PATH}"
)
