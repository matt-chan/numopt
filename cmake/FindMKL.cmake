# FindMKL.cmake. Find<package>.cmake-template from (https://cmake.org/Wiki/CMake:How_To_Find_Libraries#Writing_find_modules).

# Try to find the Math Kernel Library from Intel
# Once done, this will define
# MKL_FOUND - System has MKL
# MKL_INCLUDE_DIRS - MKL include files directories
# MKL_LIBRARIES - The MKL libraries

# We start by looking for the mkl header file. If found, we will move up in directories.
find_path(MKL_PREFIX mkl.h HINTS /opt/intel/mkl/include ${MKLROOT}/include)

if("${MKL_PREFIX}" STREQUAL "MKL_PREFIX-NOTFOUND")
    message(WARNING "MKL not found. Please set MKLROOT in environment variables in order to use MKL.")
else()
    set(MKL_FOUND TRUE)

    # Go up to MKL and INTEL directories
    get_filename_component(MKL_PREFIX ${MKL_PREFIX} DIRECTORY)
    get_filename_component(INTEL_PREFIX ${MKL_PREFIX} DIRECTORY)
    message(STATUS "MKL was found at:")
    message(STATUS ${MKL_PREFIX})
    message(STATUS ${INTEL_PREFIX})

    find_path(MKL_INCLUDE_DIRS
            NAMES mkl.h
            HINTS ${MKL_PREFIX}/include)

    find_library(MKL_INTERFACE_LIBRARY
            NAMES libmkl_intel_lp64.a
            PATHS ${MKL_PREFIX}/lib)

    find_library(MKL_INTEL_THREAD
            NAMES libmkl_intel_thread.a
            PATHS ${MKL_PREFIX}/lib)

    find_library(MKL_CORE_LIBRARY
            NAMES libmkl_core.a
            PATHS ${MKL_PREFIX}/lib)

    find_library(MKL_INTEL_OPENMP
            NAMES libiomp5.a
            PATHS ${INTEL_PREFIX}/lib)

    set(MKL_LIBRARIES ${MKL_INTERFACE_LIBRARY} ${MKL_INTEL_THREAD} ${MKL_CORE_LIBRARY} ${MKL_INTEL_OPENMP} pthread m dl)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DEIGEN_USE_MKL_ALL -DMKL_LP64 -m64")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_USE_MKL_ALL -DMKL_LP64 -m64")
endif()
