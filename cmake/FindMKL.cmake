# FindMKL.cmake. Find<package>.cmake-template from (https://cmake.org/Wiki/CMake:How_To_Find_Libraries#Writing_find_modules).

# Try to find the Math Kernel Library from Intel
# Once done, this will define
# MKL_FOUND - System has MKL
# MKL_INCLUDE_DIRS - MKL include files directories
# MKL_LIBRARIES - The MKL libraries

if(NOT DEFINED ENV{MKLROOT} AND NOT DEFINED ENV{INTEL})
    message(WARNING "MKL not found. Please set MKLROOT and INTEL in environment variables.")
else()
    message(STATUS "MKL was found.")
    set(MKL_FOUND TRUE)

    find_path(MKL_INCLUDE_DIRS
            NAMES mkl.h
            HINTS $ENV{MKLROOT}/include)

    find_library(MKL_INTERFACE_LIBRARY
            NAMES libmkl_intel_lp64.a
            PATHS $ENV{INTEL}/lib $ENV{MKLROOT}/lib)

    find_library(MKL_INTEL_THREAD
            NAMES libmkl_intel_thread.a
            PATHS $ENV{INTEL}/lib $ENV{MKLROOT}/lib)

    find_library(MKL_CORE_LIBRARY
            NAMES libmkl_core.a
            PATHS $ENV{INTEL}/lib $ENV{MKLROOT}/lib)

    find_library(MKL_INTEL_OPENMP
            NAMES libiomp5.a
            PATHS $ENV{INTEL}/lib $ENV{MKLROOT}/lib)

    set(MKL_LIBRARIES ${MKL_INTERFACE_LIBRARY} ${MKL_INTEL_THREAD} ${MKL_CORE_LIBRARY} ${MKL_INTEL_OPENMP} pthread m dl)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DEIGEN_USE_MKL_ALL -DMKL_LP64 -m64")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_USE_MKL_ALL -DMKL_LP64 -m64")
endif()
