project(tools VERSION 0.1)

###################################################
# Create library target and set properties
###################################################

# Include all header files
set(LIBRARY_TARGET_NAME ${PROJECT_NAME})
set(LIBRARY_EXPORT_NAME Tools) # <-- variable value

# Header only library, therefore INTERFACE
add_library(${LIBRARY_TARGET_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${LIBRARY_TARGET_NAME} ALIAS ${LIBRARY_TARGET_NAME})

# Interface targets only have interface properties
target_include_directories(${LIBRARY_TARGET_NAME}
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:$<$INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>
)

###################################################
# Installation instructions
###################################################
include(GNUInstallDirs)

set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${LIBRARY_EXPORT_NAME})

# `make install` to correct location
install(
        TARGETS
            ${LIBRARY_TARGET_NAME}
        EXPORT
            ${LIBRARY_TARGET_NAME}-targets
        ARCHIVE
            DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY
            DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

set_target_properties(${LIBRARY_TARGET_NAME}
        PROPERTIES
        EXPORT_NAME ${LIBRARY_EXPORT_NAME}
)

install(
        DIRECTORY
            include/
        DESTINATION
            ${CMAKE_INSTALL_INCLUDEDIR}
)

# Make project importable from the install directory
# NOTE: put config file in per-project dir (name must match),
# can also go into `shared_structs`.
install(
        EXPORT
            ${LIBRARY_TARGET_NAME}-targets
        FILE
            ${LIBRARY_TARGET_NAME}Targets.cmake
        NAMESPACE
            ${LIBRARY_TARGET_NAME}::
        DESTINATION
            ${INSTALL_CONFIGDIR}
)

# Create ConfigVersion.shared_structs file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_EXPORT_NAME}ConfigVersion.cmake
        VERSION
            ${PROJECT_VERSION}
        COMPATIBILITY
            AnyNewerVersion
)

configure_package_config_file(
        ../../cmake/scripts/${LIBRARY_EXPORT_NAME}Config.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_EXPORT_NAME}Config.cmake
        INSTALL_DESTINATION
            ${INSTALL_CONFIGDIR}
)

# Install the config, config-version, and the custom find modules
# (NOTE: only if need to like find modules if package has dependencies)
install(
        FILES
            ${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_EXPORT_NAME}Config.cmake
            ${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_EXPORT_NAME}ConfigVersion.cmake
        DESTINATION
            ${INSTALL_CONFIGDIR}
)

export(
        EXPORT
            ${LIBRARY_TARGET_NAME}-targets
        FILE
            ${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_EXPORT_NAME}Targets.cmake
        NAMESPACE
            ${LIBRARY_EXPORT_NAME}::
)