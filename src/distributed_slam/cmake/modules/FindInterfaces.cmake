# FindInterfaces.cmake
#
# Find the Interfaces library.
#
# This will define the following variables
#
#    Interfaces_FOUND
#    Interfaces_INCLUDE_DIRS
#
# and the following imported targets
#
#     Interfaces::Interfaces
#
# Author: Mason U'Ren - mason.uren600@myci.csuci.edu

######################################
# Set package specific variables
######################################
set(PACKAGE_TARGET_NAME "Interfaces")
set(TARGET_DIRECTORY_NAME "interfaces")
set(INCLUDE_HDRS
        IncidentRayInterface.h
        RoverInterface.h
)

######################################
# Generic package creation
######################################
find_package(PkgConfig REQUIRED)
pkg_check_modules(PC_${PACKAGE_TARGET_NAME} QUIET ${PACKAGE_TARGET_NAME})

find_path(
        ${PACKAGE_TARGET_NAME}_INCLUDE_DIR
        NAMES
            ${INCLUDE_HDRS}
        PATHS
            ${CMAKE_CURRENT_SOURCE_DIR}/include/${TARGET_DIRECTORY_NAME}
        PATH_SUFFIXES
            ${TARGET_DIRECTORY_NAME}
)

set(${PACKAGE_TARGET_NAME}_VERSION ${PC_${PACKAGE_TARGET_NAME}_VERSION})

mark_as_advanced(
        ${PACKAGE_TARGET_NAME}_FOUND
        ${PACKAGE_TARGET_NAME}_INCLUDE_DIR
        ${PACKAGE_TARGET_NAME}_VERSION
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
        ${PACKAGE_TARGET_NAME}
        REQUIRED_VARS
            ${PACKAGE_TARGET_NAME}_INCLUDE_DIR
        VERSION_VAR
            ${PACKAGE_TARGET_NAME}_VERSION
)

if (${PACKAGE_TARGET_NAME}_FOUND)
    #Set include dirs to parent, to enable includes like #include <PACKAGE_TARGET_NAME/document.h>
    get_filename_component(
            ${PACKAGE_TARGET_NAME}_INCLUDE_DIRS
            ${${PACKAGE_TARGET_NAME}_INCLUDE_DIR}
            DIRECTORY
    )
endif ()

if (${PACKAGE_TARGET_NAME}_FOUND AND NOT TARGET ${PACAKGE_TARGET_NAME}::${PACKAGE_TARGET_NAME})
    add_library(${PACKAGE_TARGET_NAME}::${PACKAGE_TARGET_NAME} INTERFACE IMPORTED)
    set_target_properties(
            ${PACKAGE_TARGET_NAME}::${PACKAGE_TARGET_NAME}
            PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${${PACKAGE_TARGET_NAME}_INCLUDE_DIRS}"
    )
endif ()

if (${PACKAGE_TARGET_NAME}_FOUND)
    message(STATUS "${PACKAGE_TARGET_NAME}_INCLUDE_DIR: ${${PACKAGE_TARGET_NAME}_INCLUDE_DIRS}")
#    message(STATUS "${PACKAGE_TARGET_NAME}_VERSION: ${${PACKAGE_TARGET_NAME}_VERSION}")
endif ()