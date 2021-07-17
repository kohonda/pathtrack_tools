cmake_minimum_required(VERSION 3.13)

project(mpc_tracker LANGUAGES CXX)

# set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS_DEBUG "-g -fopenmp -Wall")# Debug mode
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fopenmp -Wall")# release mode


# place binaries and libraries according to GNU standards
include(GNUInstallDirs)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

# we cannot analyse results without gcov
find_program(GCOV_PATH gcov)
if(NOT GCOV_PATH)
  message(FATAL_ERROR "Code coverage analysis requires gcov!")
endif()

# Find Eigen
find_package( Eigen3 REQUIRED )
if(${EIGEN3_FOUND})
message("-- Found Eigen version ${EIGEN3_VERSION}: ${EIGEN3_INCLUDE_DIRS}")
else()
message(STATUS "Eigen3 is NOT found")
endif()

# public include
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${EIGEN3_INCLUDE_DIR})


add_subdirectory(src)
add_subdirectory(example)

# Tests
option(PATHTRACKTOOLS_BUILD_TESTS "Build tests" ON)
message(STATUS "Build tests: ${PATHTRACKTOOLS_BUILD_TESTS}")
if (PATHTRACKTOOLS_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()  
