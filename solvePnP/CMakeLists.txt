cmake_minimum_required(VERSION 3.15)
project(solvePnP)

set(CMAKE_CXX_STANDARD 14)

include_directories( "/usr/include/eigen3" )

find_package(OpenCV REQUIRED)

#add_executable(solvePnP test.cpp )
#target_link_libraries(solvePnP ${OpenCV_LIBS})
#
#add_executable(Projectpoints Projectpoints.cpp )
#target_link_libraries(Projectpoints ${OpenCV_LIBS})
#
#add_executable(findContours findContours.cpp )
#target_link_libraries(findContours ${OpenCV_LIBS})

add_executable(thresholding thresholding.cpp )
target_link_libraries(thresholding ${OpenCV_LIBS})