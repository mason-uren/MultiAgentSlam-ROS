cmake_minimum_required(VERSION 3.0)

message(STATUS)
message(STATUS ======================)
message(STATUS "distributed_slam - SETUP")
message(STATUS "CMAKE_VERSION ${CMAKE_VERSION}")
message(STATUS ======================)

##################################################
# Setup up VERSION and CXX_STANDARD
##################################################
set(LIB_MAJOR_VERSION "1")
set(LIB_MINOR_VERSION "1")
set(LIB_PATCH_VERSION "0")
set(LIB_VERSION_STRING "${LIB_MAJOR_VERSION}.${LIB_MINOR_VERSION}.${LIB_PATCH_VERSION}")

project(distributed_slam VERSION "${LIB_VERSION_STRING}" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)

###################################################
# Set LIBRARY_TARGET_NAME and PROJECT_PATH
###################################################
set(LIBRARY_TARGET_NAME ${PROJECT_NAME})
set(PROJECT_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

###################################################
# Set CMAKE_MODULE_PATH so custom make files
#   can be found.
###################################################
list(INSERT CMAKE_MODULE_PATH 0 ${PROJECT_PATH}/cmake/modules)
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

###################################################
# Find external libraries/headers
###################################################
#add_subdirectory(${PROJECT_PATH}/include/interfaces)
#add_subdirectory(${PROJECT_PATH}/include/nholmann_json)
#add_subdirectory(${PROJECT_PATH}/include/shared_structs)
#add_subdirectory(${PROJECT_PATH}/include/templates)

include("cmake/modules/Boost.cmake")
include("cmake/modules/FindEigen3.cmake")
include("cmake/modules/FindMKL.cmake")

#if (${CMAKE_VERSION} GREATER 3.0)
#    cmake_policy(SET CMP0074 NEW)
#endif()

find_package(MKL MODULE REQUIRED)
find_package(Interfaces REQUIRED)
find_package(JSON REQUIRED)
find_package(SharedStructs REQUIRED)
find_package(Templates REQUIRED)

find_package(catkin REQUIRED
        COMPONENTS
            ros_slam_msgs
)

catkin_package(
        INCLUDE_DIRS
            include
        LIBRARIES
            ${LIBRARY_TARGET_NAME}
        CATKIN_DEPENDS
            ros_slam_msgs
        DEPENDS
            Boost
            MKL
            Interfaces
            JSON
            SharedStructs
            Templates
)

###################################################
# Create library target and set properties
###################################################
set(
        PROJECT_SOURCES
        ${CMAKE_CURRENT_LIST_DIR}/src/Agent/Detections/Detection.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Agent/Moments/Moments.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Agent/Rover/Rover.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Agent/Rover/RoverFactory.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Slam/FeatureSet/FeatureSet.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Slam/Seif/Seif.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Slam/SlamAdapter/SlamAdapter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Utilities/ActiveRovers/ActiveRovers.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Utilities/ConfigParser/ConfigParser.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Utilities/Equations/Equations.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Utilities/RedBlackTree/RedBlackTree.cpp
)

add_library(
        ${LIBRARY_TARGET_NAME} src/Adapter.cpp ${PROJECT_SOURCES}
)

# Catkin dependencies
add_dependencies(${LIBRARY_TARGET_NAME}
        ${${LIBRARY_TARGET_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

# Set target properties
target_include_directories(
        # Target
        ${LIBRARY_TARGET_NAME}
        PUBLIC
            ${catkin_INCLUDE_DIRS}
            ${Boost_INCLUDE_DIRS}
            ${Eigen3_INCLUDE_DIRS}
            ${MKL_INCLUDE_DIRS}
            ${Interfaces_INCLUDE_DIRS}
            ${JSON_INCLUDE_DIRS}
            ${SharedStructs_INCLUDE_DIRS}
            ${Templates_INCLUDE_DIRS}
)
target_link_libraries(${LIBRARY_TARGET_NAME}
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${Eigen3_LIBRARIES}
        ${MKL_LIBRARIES}
        ${Interfaces_LIBARARIES}
        ${JSON_LIBRARIES}
        ${SharedStructs_LIBRARIES}
        ${Templates_LIBRARIES}
)

###################################################
# Installation instructions
###################################################
install(
        TARGETS
            ${LIBRARY_TARGET_NAME}
        ARCHIVE
            DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY
            DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME
            DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
        DIRECTORY
            include/${LIBRARY_TARGET_NAME}
        DESTINATION
            ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN
            ".svn" EXCLUDE
)

###################################################
# Export from the build tree
###################################################

message(STATUS ======================)
message(STATUS "distributed_slam - FINISHED")
message(STATUS ======================)
message(STATUS)



