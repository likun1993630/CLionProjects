cmake_minimum_required(VERSION 3.15)
project(ArucoTracker)

set(CMAKE_CXX_STANDARD 14)
# 允许c++11标准、O3优化、多线程。match选项可避免一些cpu上的问题
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -O3 -pthread" )

find_package(OpenCV REQUIRED)

include_directories(.)
add_executable(ArucoTracker ArucoTracker.cpp KalmanFilterAruco.cpp)
target_link_libraries( ArucoTracker ${OpenCV_LIBS})