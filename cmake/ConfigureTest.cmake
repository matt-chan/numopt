# In this CMake file, we will include header files and link to libraries for a given test source


# Include this project
target_include_directories(${TEST_NAME} PRIVATE ${PROJECT_INCLUDE_FOLDER})
target_link_libraries(${TEST_NAME} PRIVATE ${LIBRARY_NAME})

# Include Boost
target_include_directories(${TEST_NAME} PUBLIC ${Boost_INCLUDE_DIRS})

# Include Spectra
target_include_directories(${TEST_NAME} PRIVATE ${Spectra_INCLUDE_DIRS})

# Include cpputil
target_include_directories(${TEST_NAME} PRIVATE ${cpputil_INCLUDE_DIRS})
target_link_libraries(${TEST_NAME} PRIVATE cpputil)

# Include MKL (optional)
if (MKL_FOUND)
    target_include_directories(${TEST_NAME} PRIVATE ${MKL_INCLUDE_DIRS})
    target_link_libraries(${TEST_NAME} PRIVATE ${MKL_LIBRARIES})
endif()
