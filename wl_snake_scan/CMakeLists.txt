cmake_minimum_required(VERSION 2.8.3)
project(scan)
find_package(OpenCV REQUIRED)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg)

## Declare a catkin package
catkin_package()

## Build listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(scan src/scan.cpp)
target_link_libraries(scan ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})