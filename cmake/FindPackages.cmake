# In this CMake file, we will find all required packages


# Find the boost package - needed for unittests
find_package(Boost REQUIRED)

# Find eigen
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Find cpputil
find_package(cpputil 1.2.0 REQUIRED)
