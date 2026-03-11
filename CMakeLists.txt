cmake_minimum_required(VERSION 3.10)
project(LaneKeepingMPC)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OsqpEigen_INCLUDE_DIRS}
    ${LIBSERIAL_INCLUDE_DIRS}
)

link_directories(
    ${LIBSERIAL_LIBRARY_DIRS}
)

set(SOURCES
    src/main.cpp
    src/LaneDetector.cpp
    src/MpcController.cpp
    src/communication.cpp
    src/logger.cpp
    src/Trans_UDP.cpp
    src/logic.cpp
)

add_executable(hihi ${SOURCES})

target_link_libraries(hihi
    ${OpenCV_LIBS}
    Eigen3::Eigen
    OsqpEigen::OsqpEigen
    pthread
    ${LIBSERIAL_LIBRARIES}
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

message(STATUS "==========================================")
message(STATUS "Lane Keeping MPC Configuration Summary")
message(STATUS "==========================================")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "Eigen3 include: ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "OsqpEigen include: ${OsqpEigen_INCLUDE_DIRS}")
message(STATUS "LibSerial include: ${LIBSERIAL_INCLUDE_DIRS}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Output directory: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "==========================================")