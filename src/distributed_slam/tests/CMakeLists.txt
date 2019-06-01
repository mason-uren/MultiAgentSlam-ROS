cmake_minimum_required(VERSION 3.0)

message(STATUS "======================")
message(STATUS "Building... Test Environment.")
message(STATUS ======================)

##################################################
# Setup up VERSION and CXX_STANDARD
##################################################
set(LIB_MAJOR_VERSION "1")
set(LIB_MINOR_VERSION "1")
set(LIB_PATCH_VERSION "0")
set(LIB_VERSION_STRING "${LIB_MAJOR_VERSION}.${LIB_MINOR_VERSION}.${LIB_PATCH_VERSION}")

project(test_environment VERSION "${LIB_VERSION_STRING}" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)

###################################################
# Set:
# -- LIBRARY_TARGET_NAME
# -- PROJECT_PATH
# -- CMAKE_INSTALL_PREFIX
# -- CMAKE_BINARY_DIR
###################################################
set(LIBRARY_TARGET_NAME "utilities_test")
set(PROJECT_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_LIST_DIR})
set(CMAKE_BINARY_DIR ${CMAKE_CURRENT_LIST_DIR}/bin)

message(STATUS "Project Path - ${PROJECT_PATH}")
message(STATUS "Install Path - ${CMAKE_BINARY_DIR}")

###################################################
# Set CMAKE_MODULE_PATH so custom make files
#   can be found.
###################################################
#list(INSERT CMAKE_MODULE_PATH 0 ${PROJECT_PATH}cmake/modules)
#message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

###################################################
# Find external libraries/headers
###################################################
find_package(Boost REQUIRED
    COMPONENTS
        filesystem
        system
)

# Boost Package Properties
#set(-DBOOST_NO_CXX11_SCOPED_ENUMS TRUE)

###################################################
# Create executable target and set properties
# Executables:
# -- Utilities Test
###################################################
set(
        PROJECT_SOURCES
        ${PROJECT_PATH}/src/Utilities/Equations/Equations.cpp
        ${PROJECT_PATH}/src/Logger.cpp
)

# Utilites Executable
add_executable(
        ${LIBRARY_TARGET_NAME} ${CMAKE_CURRENT_LIST_DIR}/Utilities/utilsTest.cpp ${PROJECT_SOURCES}
)

target_include_directories(
        # Target
        ${LIBRARY_TARGET_NAME}
        PUBLIC
            include
            ${Boost_INCLUDE_DIRS}

)
target_link_libraries(${LIBRARY_TARGET_NAME}
        ${Boost_LIBRARIES}
)

###################################################
# Installation instructions
###################################################
install(
        TARGETS
            ${LIBRARY_TARGET_NAME}
        ARCHIVE
            DESTINATION lib
        LIBRARY
            DESTINATION lib
        RUNTIME
            DESTINATION ${CMAKE_BINARY_DIR}
)

install(
        DIRECTORY
            include/${LIBRARY_TARGET_NAME}
        DESTINATION
            ${CMAKE_BINARY_DIR}
        FILES_MATCHING
            PATTERN "*.h"
            PATTERN ".svn" EXCLUDE
)

###################################################
# Export from the build tree
###################################################

message(STATUS ======================)
message(STATUS "Finished Build. Test Environment.")
message(STATUS ======================)



