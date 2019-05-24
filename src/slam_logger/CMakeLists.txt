cmake_minimum_required(VERSION 3.0)

##################################################
# Setup up VERSION and CXX_STANDARD
##################################################
set(LIB_MAJOR_VERSION "1")
set(LIB_MINOR_VERSION "1")
set(LIB_PATCH_VERSION "0")
set(LIB_VERSION_STRING "${LIB_MAJOR_VERSION}.${LIB_MINOR_VERSION}.${LIB_PATCH_VERSION}")

project(slam_logger VERSION "${LIB_VERSION_STRING}" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)

###################################################
# Set LIBRARY_TARGET_NAME and PROJECT_PATH
###################################################
set(LIBRARY_TARGET_NAME ${PROJECT_NAME})
set(PROJECT_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

###################################################
# Find external libraries/headers
###################################################
# Boost cannot be a catkin component
find_package(Boost REQUIRED)

find_package(catkin REQUIRED
    COMPONENTS
        distributed_slam
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${LIBRARY_TARGET_NAME}
    CATKIN_DEPENDS
        distributed_slam
    DEPENDS
        Boost
)

###################################################
# Create library target and set properties
###################################################
set(
    PROJECT_SOURCES
)

add_library(
        ${LIBRARY_TARGET_NAME} src/Logger.cpp ${PROJECT_SOURCES}
)

#Catkin dependencies
add_dependencies(${LIBRARY_TARGET_NAME}
        ${${LIBRARY_TARGET_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

# Set target properties
target_include_directories(${LIBRARY_TARGET_NAME}
        PUBLIC
            ${catkin_INCLUDE_DIRS}
            ${Boost_INCLUDE_DIRS}
)

target_link_libraries(${LIBRARY_TARGET_NAME}
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
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