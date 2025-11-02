cmake_minimum_required(VERSION 3.10)
project(LaneKeepingMPC)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp QUIET)
find_package(OsqpEigen QUIET)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

if(osqp_FOUND)
    include_directories(${osqp_INCLUDE_DIRS})
else()
    include_directories(/usr/local/include/osqp)
endif()

if(OsqpEigen_FOUND)
    include_directories(${OsqpEigen_INCLUDE_DIRS})
else()
    include_directories(/usr/local/include/OsqpEigen)
endif()

# Source files (CHÃš Ã: KhÃ´ng cÃ³ MpcController.cpp ná»¯a!)
set(SOURCES
    src/main.cpp
    src/LaneDetector.cpp
    src/MpcController.cpp
)

# Header files
set(HEADERS
    include/LaneDetector.hpp
    include/MpcController.hpp
)

# Create executable
add_executable(lane_keeping_mpc ${SOURCES} ${HEADERS})

# Link libraries
target_link_libraries(lane_keeping_mpc
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

if(osqp_FOUND)
    target_link_libraries(lane_keeping_mpc osqp::osqp)
else()
    find_library(OSQP_LIB osqp PATHS /usr/local/lib)
    target_link_libraries(lane_keeping_mpc ${OSQP_LIB})
endif()

if(OsqpEigen_FOUND)
    target_link_libraries(lane_keeping_mpc OsqpEigen::OsqpEigen)
else()
    find_library(OSQPEIGEN_LIB OsqpEigen PATHS /usr/local/lib)
    target_link_libraries(lane_keeping_mpc ${OSQPEIGEN_LIB})
endif()

target_link_libraries(lane_keeping_mpc pthread)

target_compile_options(lane_keeping_mpc PRIVATE 
    -O3 
    -march=native
    -Wall
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)