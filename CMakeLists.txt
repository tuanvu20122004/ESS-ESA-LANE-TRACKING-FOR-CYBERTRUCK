cmake_minimum_required(VERSION 3.16)
project(LaneMPC_Project CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Nếu bạn dùng MSYS2 trên Windows
list(APPEND CMAKE_PREFIX_PATH "C:/msys64/mingw64")

# ==== OpenCV ====
find_package(OpenCV REQUIRED CONFIG)

# ==== OSQP-Eigen ====
find_package(OsqpEigen REQUIRED)

# ==== Eigen ====
find_package(Eigen3 REQUIRED)

# ==== LibSerial ====
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# ==== Thư viện chung (class, hàm) ====
add_library(cv_core
    src/LaneDetector.cpp
    src/MpcController.cpp
    src/ComputeMpc.cpp
    src/communication.cpp
    src/logger.cpp
    src/Trans_UDP.cpp
    src/logic.cpp
)

target_include_directories(cv_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OsqpEigen_INCLUDE_DIRS}
    ${LIBSERIAL_INCLUDE_DIRS}
)

target_link_libraries(cv_core PUBLIC
    ${OpenCV_LIBS}
    Eigen3::Eigen
    OsqpEigen::OsqpEigen
    ${LIBSERIAL_LIBRARIES}
    ws2_32         # cần cho Windows socket
    pthread
)

# ==== Chương trình chính ====
add_executable(main_app src/main.cpp)

target_link_libraries(main_app PRIVATE cv_core)

# ==== Thư mục output ====
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ==== Thông tin tóm tắt ====
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
