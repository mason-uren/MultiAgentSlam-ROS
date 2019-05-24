cmake_minimum_required(VERSION 2.8.3)
project(behaviours)

SET(CMAKE_CXX_FLAGS "-std=c++11")

# Make sure that custom modules are found
#list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/distributed_slam/cmake/modules)
#
#set(distributed_slam_DIR ${CMAKE_CURRENT_BINARY_DIR})

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
    random_numbers
    tf
    apriltags_ros
    swarmie_msgs
    ros_slam_msgs
    distributed_slam
    slam_logger
)

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        swarmie_msgs
        roscpp
        sensor_msgs
        std_msgs
        random_numbers
        tf
        apriltags_ros
        ros_slam_msgs
        distributed_slam
        slam_logger
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(
  behaviours
  src/Tag.cpp
  src/ObstacleController.cpp 
  src/PickUpController.cpp
  src/DropOffController.cpp
  src/SearchController.cpp
  src/ROSAdapter.cpp
  src/PID.cpp
  src/DriveController.cpp
  src/RangeController.cpp
  src/LogicController.cpp
  src/ManualWaypointController.cpp
  src/PositionPublisher.cpp
)

add_dependencies(
        behaviours
        ${catkin_EXPORTED_TARGETS}

)

target_link_libraries(
  behaviours
    ${catkin_LIBRARIES}
        rt  # <--- Inclusion causes the simulator to build rover incorrectly!!!
)

