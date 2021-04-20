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

add_executable(simple_navigation_goals src/simple_navigation_goals.cpp)
target_link_libraries(simple_navigation_goals ${catkin_LIBRARIES})
