cmake_minimum_required(VERSION 3.16)
project(hello_cv CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Để CMake tìm được file config của OpenCV trong MSYS2
list(APPEND CMAKE_PREFIX_PATH "C:/msys64/mingw64")

find_package(OpenCV REQUIRED CONFIG)

# ---- Thư viện chung (class, hàm) ----
add_library(cv_core
    LaneDetector.cpp
    ComputeMpc.cpp
)

target_include_directories(cv_core PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(cv_core PUBLIC ${OpenCV_LIBS})

# ---- Các executable (nhiều main) ----
add_executable(app1 main.cpp)
target_link_libraries(app1 PRIVATE cv_core)
