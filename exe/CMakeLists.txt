# exe-level CMakeLists

# Add an executable for davidson timings
add_executable(davidson_timings ${PROJECT_EXECUTABLES_FOLDER}/davidson_timings.cpp)


# Add this project's headers
target_include_directories(davidson_timings PRIVATE ${PROJECT_INCLUDE_FOLDER})

# Link to this project's library
target_link_libraries(davidson_timings PRIVATE ${LIBRARY_NAME})

# Include Eigen
target_link_libraries(davidson_timings PRIVATE Eigen3::Eigen)

# Include cpputil
target_include_directories(davidson_timings PRIVATE ${cpputil_INCLUDE_DIRS})
target_link_libraries(davidson_timings PRIVATE cpputil)
