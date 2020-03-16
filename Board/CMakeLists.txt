cmake_minimum_required(VERSION 3.15)
project(Board)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)
add_executable(Board main.cpp)
target_link_libraries(Board ${OpenCV_LIBS})