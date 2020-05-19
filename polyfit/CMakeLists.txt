cmake_minimum_required(VERSION 3.15)
project(polyfit)
set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

add_executable(polyfit main.cpp)
target_link_libraries(polyfit ${OpenCV_LIBS} )