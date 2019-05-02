find_package (Eigen3 3.3 REQUIRED NO_MODULE)

set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
set(EIGEN3_LIBRARIES ${EIGEN3_LIBRARIES})

if(EIGEN3_FOUND)
    message(STATUS "Eigen3_VERSION: ${Eigen3_VERSION}")
    message(STATUS "Eigen3_INCLUDE_DIRS: ${EIGEN3_INCLUDE_DIRS}")
    message(STATUS "Eigen3_LIBRARIES: ${EIGEN3_LIBRARIES}")
endif()

if (NOT Eigen3_FOUND)
    message(FATAL_ERROR "Could not find Eigen.")
endif()

include_directories(
        ${EIGEN3_INCLUDE_DIRS}
        ${EIGEN3_LIBRARIES}
)