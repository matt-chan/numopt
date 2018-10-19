# In this CMake file, we will include the headers and link to the necessary libraries


# Include this project
target_include_directories(${LIBRARY_NAME} PRIVATE ${PROJECT_INCLUDE_FOLDER})

# Include Eigen
target_link_libraries(${LIBRARY_NAME} PUBLIC Eigen3::Eigen)

# Include Spectra
target_include_directories(${LIBRARY_NAME} PRIVATE ${Spectra_INCLUDE_DIRS})

# Include cpputil
target_include_directories(${LIBRARY_NAME} PRIVATE ${cpputil_INCLUDE_DIRS})
target_link_libraries(${LIBRARY_NAME} PRIVATE cpputil)

# Include MKL (optional)
if (MKL_FOUND)
    target_include_directories(${LIBRARY_NAME} PRIVATE ${MKL_INCLUDE_DIRS})
    target_link_libraries(${LIBRARY_NAME} PRIVATE ${MKL_LIBRARIES})
endif()
