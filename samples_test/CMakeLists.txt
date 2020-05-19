cmake_minimum_required(VERSION 3.15)
project(samples_test)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)

#add_executable(samples_test main.cpp)
#target_link_libraries(samples_test ${OpenCV_LIBS})

add_executable(main2 main2.cpp)
target_link_libraries(main2 ${OpenCV_LIBS})