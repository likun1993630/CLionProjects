cmake_minimum_required(VERSION 3.15)
project(VideoCap)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV 3.4.8 REQUIRED)
add_executable(VideoCap main.cpp)
target_link_libraries(VideoCap ${OpenCV_LIBS})