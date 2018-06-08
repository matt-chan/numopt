# In this CMake file, all CMake variables will be set.



# Parse the project name into uppercase and lowercase
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPERCASE)  # uppercase is needed in version.hpp.in
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWERCASE)



# The name of the library should be equal to the project name
if(NOT LIBRARY_NAME)
    set(LIBRARY_NAME ${PROJECT_NAME})
endif()

# We want to make a static library
set(LIBRARY_TYPE STATIC)
set(EXPORT_TYPE ARCHIVE)



# Find the source folder
set(PROJECT_SOURCE_FOLDER ${CMAKE_SOURCE_DIR}/src)

# Find the source files
set(PROJECT_SOURCE_FILES
        ${PROJECT_SOURCE_FOLDER}/BaseEigenproblemSolver.cpp
        ${PROJECT_SOURCE_FOLDER}/BaseMatrixSolver.cpp
        ${PROJECT_SOURCE_FOLDER}/BaseMinimizer.cpp
        ${PROJECT_SOURCE_FOLDER}/BaseSystemOfEquationsSolver.cpp
        ${PROJECT_SOURCE_FOLDER}/DavidsonSolver.cpp
        ${PROJECT_SOURCE_FOLDER}/DenseSolver.cpp
        ${PROJECT_SOURCE_FOLDER}/Eigenpair.cpp
        ${PROJECT_SOURCE_FOLDER}/NewtonMinimizer.cpp
        ${PROJECT_SOURCE_FOLDER}/NewtonSystemOfEquationsSolver.cpp
        ${PROJECT_SOURCE_FOLDER}/SparseSolver.cpp
        ${PROJECT_SOURCE_FOLDER}/step.cpp)

# Find the header folder
set(PROJECT_INCLUDE_FOLDER ${CMAKE_SOURCE_DIR}/include)

# Find the header files (not including version.hpp.in)
set(PROJECT_INCLUDE_FILES
        ${PROJECT_INCLUDE_FOLDER}/BaseEigenproblemSolver.hpp
        ${PROJECT_INCLUDE_FOLDER}/BaseMatrixSolver.hpp
        ${PROJECT_INCLUDE_FOLDER}/BaseMinimizer.hpp
        ${PROJECT_INCLUDE_FOLDER}/BaseSystemOfEquationsSolver.hpp
        ${PROJECT_INCLUDE_FOLDER}/common.hpp
        ${PROJECT_INCLUDE_FOLDER}/DavidsonSolver.hpp
        ${PROJECT_INCLUDE_FOLDER}/DenseSolver.hpp
        ${PROJECT_INCLUDE_FOLDER}/Eigenpair.hpp
        ${PROJECT_INCLUDE_FOLDER}/EigenproblemSolverOptions.hpp
        ${PROJECT_INCLUDE_FOLDER}/NewtonMinimizer.hpp
        ${PROJECT_INCLUDE_FOLDER}/NewtonSystemOfEquationsSolver.hpp
        ${PROJECT_INCLUDE_FOLDER}/numopt.hpp
        ${PROJECT_INCLUDE_FOLDER}/SparseSolver.hpp
        ${PROJECT_INCLUDE_FOLDER}/step.hpp
        ${PROJECT_INCLUDE_FOLDER}/version.hpp)

# Find the tests folder
set(PROJECT_TESTS_FOLDER ${CMAKE_SOURCE_DIR}/tests)

# Find the source files for the tests
set(PROJECT_TEST_SOURCE_FILES
        ${PROJECT_TESTS_FOLDER}/DavidsonSolver_test.cpp
        ${PROJECT_TESTS_FOLDER}/DenseSolver_test.cpp
        ${PROJECT_TESTS_FOLDER}/Eigenpair_test.cpp
        ${PROJECT_TESTS_FOLDER}/NewtonMinimizer_test.cpp
        ${PROJECT_TESTS_FOLDER}/NewtonSystemOfEquationsSolver_test.cpp
        ${PROJECT_TESTS_FOLDER}/SparseSolver_test.cpp)

# Find the folder that has sources for executables
set(PROJECT_EXECUTABLES_FOLDER ${CMAKE_SOURCE_DIR}/exe)



# Give the user the option to specify an installation prefix. If not given as -DINSTALLATION_PREFIX, defaults to
# /usr/local
if(NOT INSTALLATION_PREFIX)
    set(INSTALLATION_PREFIX ${CMAKE_INSTALL_PREFIX})
endif()
set(PROJECT_INSTALL_DIR ${INSTALLATION_PREFIX}/${PROJECT_NAME_LOWERCASE})
set(INCLUDE_INSTALL_DIR ${PROJECT_INSTALL_DIR}/include)
set(CMAKE_INSTALL_DIR ${PROJECT_INSTALL_DIR}/cmake)
set(LIBRARY_INSTALL_DIR ${PROJECT_INSTALL_DIR}/lib)
