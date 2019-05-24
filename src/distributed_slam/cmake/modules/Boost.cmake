# BOOST
find_package(Boost REQUIRED)

set(Boost_INCLUDE_DIRS ${Boost_INCLUDE_DIRS})
set(Boost_LIBRARY_DIRS ${Boost_LIBRARY_DIRS})

if (Boost_FOUND)
    message(STATUS "Boost_INCLUDE_DIR: ${Boost_INCLUDE_DIRS}")
    message(STATUS "Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}")
#    message(STATUS "Boost_VERSION: ${Boost_VERSION}")
endif ()
if(NOT Boost_FOUND)
    message(FATAL_ERROR "Could not find boost!")
endif()

include_directories(
        ${Boost_INCLUDE_DIRS}
)
link_directories(
        ${Boost_LIBRARY_DIRS}
)