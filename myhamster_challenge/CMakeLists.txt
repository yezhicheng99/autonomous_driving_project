cmake_minimum_required(VERSION 2.8.3)
project(myhamster_challenge)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    rospy 
    std_msgs 
    genmsg 
    move_base_msgs
    actionlib
    actionlib_msgs)

## Declare ROS messages and services


## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
    LIBRARIES myhamster_challenge
    CATKIN_DEPENDS actionlib actionlib_msgs roscpp std_msgs
    DEPENDS system_lib
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(reverse_parking src/reverse_parking.cpp)
target_link_libraries(reverse_parking ${catkin_LIBRARIES})

add_executable(right_angle src/right_angle.cpp)
target_link_libraries(right_angle ${catkin_LIBRARIES})

add_executable(side_parking src/side_parking.cpp)
target_link_libraries(side_parking ${catkin_LIBRARIES})
