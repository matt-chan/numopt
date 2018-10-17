# numopt v1.5.0

[![Build Status](https://travis-ci.org/GQCG/numopt.svg?branch=master)](https://travis-ci.org/GQCG/numopt)

A C++ library for performing numerical optimization.



## Dependencies
[![Boost Dependency](https://img.shields.io/badge/Boost-1.65.1+-000000.svg)](www.boost.org)
[![Eigen3 Dependency](https://img.shields.io/badge/Eigen-3.3.4+-000000.svg)](http://eigen.tuxfamily.org/index.php?title=Main_Page)
[![cpputil Dependency](https://img.shields.io/badge/cpputil-1.2.2+-blue.svg)](https://github.com/GQCG/cpputil)



## Installation
To install this library:
1. clone the master branch, which contains the latest release:

        git clone https://github.com/GQCG/numopt.git --branch master --single-branch --recurse-submodules
        cd numopt

2. perform an out-of-source cmake build:

        mkdir build && cd build
        cmake -DINSTALLATION_PREFIX=prefix ..
        make && make test && sudo make install

    where
    * `prefix` is the installation prefix (defaulted to `/usr/local`) you want the library to be installed at:
        * the library `libnumopt.a` will be installed in `prefix/numopt/lib`
        * the header files (and cmake files, see Usage) will be installed in `prefix/numopt/include`



## Usage
Basic usage of this library can be found in the `tests` directory. If you use CMake in other projects, you can add the following CMake command to the CMakeLists.txt-file:

    find_package(numopt 1.4.0)

CMake then provides the commands `numopt_INCLUDE_DIRS` to be used in your `target_include_directories` and the library `numopt` to be used in your `target_link_libraries`.
