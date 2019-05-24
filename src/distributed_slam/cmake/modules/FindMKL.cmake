# - Find Intel MKL
# Find the MKL libraries
#
# Options:
#
#   MKL_ROOT       :   root path of MKL.
#   MKLROOT       :   root path of MKL.
#
# This module defines the following variables:
#
#   MKL_FOUND            : True if MKL_INCLUDE_DIR are found
#   MKL_INCLUDE_DIR      : where to find mkl.h, etc.
#   MKL_INCLUDE_DIRS     : set when MKL_INCLUDE_DIR found
#   MKL_LIBRARIES        : the library to link against.
include(FindPackageHandleStandardArgs)

if (UNIX AND NOT APPLE)
    set(MKL_ROOT /opt/intel/compilers_and_libraries_2019.3.199/linux/mkl)
elseif(APPLE)
    set(MKL_ROOT /opt/intel/compilers_and_libraries_2019.3.199/mac/mkl/)
endif()


if((NOT MKL_ROOT) AND (DEFINED ENV{MKLROOT}))
    set(MKL_ROOT $ENV{MKLROOT} CACHE PATH "Folder contains MKL")
endif()

if(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    if (APPLE)
        set(MKL_ARCH_DIR "${MKL_ROOT}/lib")
    else()
        set(MKL_ARCH_DIR "${MKL_ROOT}/lib/intel64")
    endif ()
    set(MKL_INTERFACE_LIBNAME mkl_intel_lp64)
else()
    set(MKL_ARCH_DIR "${MKL_ROOT}/lib/ia32")
    set(MKL_INTERFACE_LIBNAME mkl_intel)
endif()

# Find include dir
find_path(
        MKL_INCLUDE_DIRS
            fftw3.h
        PATHS
            ${MKL_ROOT}
        PATH_SUFFIXES
            "include"
            "include/fftw"
        NO_DEFAULT_PATH
)

find_library(
        MKL_INTERFACE_LIBRARY
            ${MKL_INTERFACE_LIBNAME}
        PATHS
            ${MKL_ARCH_DIR})

find_library(MKL_THREADING_LIBRARY mkl_sequential
        PATHS ${MKL_ARCH_DIR})

find_library(MKL_CORE_LIBRARY mkl_core
        PATHS ${MKL_ARCH_DIR})

set(MKL_LIBRARIES ${MKL_INTERFACE_LIBRARY} ${MKL_THREADING_LIBRARY} ${MKL_CORE_LIBRARY})
set(MKL_VERSION $})

find_package_handle_standard_args(MKL DEFAULT_MSG MKL_INCLUDE_DIRS MKL_LIBRARIES)

mark_as_advanced(
        MKL_INCLUDE_DIRS
        MKL_LIBRARIES
        MKL_VERSION
)

if(MKL_FOUND)
    message(STATUS "MKL_ROOT: ${MKL_ROOT}")
    message(STATUS "MKL_INCLUDE_DIR: ${MKL_INCLUDE_DIRS}")
    message(STATUS "MKL_LIBRARIES: ${MKL_LIBRARIES}")
endif()
