cmake_minimum_required(VERSION 3.15)
project(Test)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)

#add_executable(Test main.cpp)
#target_link_libraries( Test ${OpenCV_LIBS})

add_executable(MarkerCreater MarkerCreater.cpp)
target_link_libraries(MarkerCreater ${OpenCV_LIBS})