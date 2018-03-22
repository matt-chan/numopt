# In this CMake file, we will include the headers and link to the necessary libraries


# Include this project's headers
target_include_directories(${LIBRARY_NAME} PRIVATE ${PROJECT_INCLUDE_FOLDER})

# Include Eigen3
target_link_libraries(${LIBRARY_NAME} PUBLIC Eigen3::Eigen)

# Include Spectra
target_include_directories(${LIBRARY_NAME} PRIVATE ${spectra_INCLUDE_DIRS})
