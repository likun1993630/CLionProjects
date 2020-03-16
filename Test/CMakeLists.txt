cmake_minimum_required(VERSION 3.15)
project(Test)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV 3.4 REQUIRED)

add_executable(MarkerCreater MarkerCreater.cpp)
target_link_libraries(MarkerCreater  ${OpenCV_LIBS})
#add_dependencies(MarkerCreater ${${PROJECT_NAME}_EXPORTED_TARGETS})