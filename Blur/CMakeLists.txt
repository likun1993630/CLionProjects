cmake_minimum_required(VERSION 3.15)
project(Blur)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

add_executable(Blur main.cpp)
target_link_libraries(Blur ${OpenCV_LIBS})