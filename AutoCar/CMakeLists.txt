cmake_minimum_required(VERSION 3.10)
project(LaneKeepingMPC)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Try to find OSQP and OsqpEigen
find_package(osqp QUIET)
find_package(OsqpEigen QUIET)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# Add OSQP include directories if found
if(osqp_FOUND)
    include_directories(${osqp_INCLUDE_DIRS})
    message(STATUS "Found osqp package")
else()
    include_directories(/usr/local/include/osqp)
    message(STATUS "Using manual osqp path: /usr/local/include/osqp")
endif()

# Add OsqpEigen include directories if found
if(OsqpEigen_FOUND)
    include_directories(${OsqpEigen_INCLUDE_DIRS})
    message(STATUS "Found OsqpEigen package")
else()
    include_directories(/usr/local/include/OsqpEigen)
    message(STATUS "Using manual OsqpEigen path: /usr/local/include/OsqpEigen")
endif()

# Source files
set(SOURCES
    src/main.cpp
    src/LaneDetector.cpp
    src/ComputeMpc.cpp
    src/MpcController.cpp
)

# Header files (for IDE support)
set(HEADERS
    include/LaneDetector.hpp
    include/ComputeMpc.hpp
    include/MpcController.hpp
    include/MpcState.hpp
    include/MpcParameters.hpp
)

# Create executable
add_executable(lane_keeping_mpc ${SOURCES} ${HEADERS})

# Link libraries
target_link_libraries(lane_keeping_mpc
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

# Link OSQP
if(osqp_FOUND)
    target_link_libraries(lane_keeping_mpc osqp::osqp)
else()
    find_library(OSQP_LIB osqp PATHS /usr/local/lib)
    if(OSQP_LIB)
        target_link_libraries(lane_keeping_mpc ${OSQP_LIB})
        message(STATUS "Found osqp library: ${OSQP_LIB}")
    else()
        message(FATAL_ERROR "Could not find osqp library")
    endif()
endif()

# Link OsqpEigen
if(OsqpEigen_FOUND)
    target_link_libraries(lane_keeping_mpc OsqpEigen::OsqpEigen)
else()
    find_library(OSQPEIGEN_LIB OsqpEigen PATHS /usr/local/lib)
    if(OSQPEIGEN_LIB)
        target_link_libraries(lane_keeping_mpc ${OSQPEIGEN_LIB})
        message(STATUS "Found OsqpEigen library: ${OSQPEIGEN_LIB}")
    else()
        message(FATAL_ERROR "Could not find OsqpEigen library")
    endif()
endif()

# Link pthread
target_link_libraries(lane_keeping_mpc pthread)

# Compiler flags
target_compile_options(lane_keeping_mpc PRIVATE 
    -O3 
    -march=native
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wno-unused-variable
)

# Set output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Print configuration summary
message(STATUS "==========================================")
message(STATUS "Lane Keeping MPC Configuration Summary")
message(STATUS "==========================================")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "Eigen3 include: ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Output directory: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "==========================================")