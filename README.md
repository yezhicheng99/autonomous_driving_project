
# Autonomous driving project for ackermann steering/car-like vehicle using ROS

## Simulation
<img src="https://github.com/yezhicheng99/autonomous_driving_project/blob/master/readme/env.png" width="900">

## Curve Driving
<img src="https://github.com/yezhicheng99/autonomous_driving_project/blob/master/readme/s_shape.gif" width="500">

## Reverse Parking
<p float="left">
  <img src="https://github.com/yezhicheng99/autonomous_driving_project/blob/master/readme/reverse_parking_speed.gif" width="400" />
  <img src="https://github.com/yezhicheng99/autonomous_driving_project/blob/master/readme/reverse_parking_2_speed.gif" width="400" />
</p>

## Side Parking
<img src="https://github.com/yezhicheng99/autonomous_driving_project/blob/master/readme/side_parking_speed.gif" width="500">


Tested on:
- Ubuntu 18.04 (ROS Melodic)

## 1. Installation

### 1.1 Clone and install all dependencies:

    sudo apt install -y python-rosdep
    cd <your_ws>/src
    git clone https://github.com/yezhicheng99/autonomous_driving_project.git
    cd ..
    rosdep install --from-paths src --ignore-src -r -y

### 1.2 Build your workspace:

    cd <your_ws>
    catkin_make
    source <your_ws/>/devel/setup.bash

## 2. Quick Start


### 2.1 SLAM demo:

<img src="https://github.com/yezhicheng99/autonomous_driving_project/blob/master/readme/slam.png" width="500">

#### 2.1.1 Run SLAM launch file:

    roslaunch myhamster_slam myhamster_slam.launch slam_methods:=gmapping

    Change the "slam_methods" value as you want: [gmapping, cartographer, hector, karto]

#### 2.1.2 Run the teleop node:

    roslaunch mytutorial_pkg teleop.launch

#### 2.1.3 Save map
    cd <your_ws>/src/autonomous_driving_project/myhamster_navigation/maps
    rosrun map_server map_saver -f map


### 2.2 Autonomous Navigation:

<img src="https://github.com/yezhicheng99/autonomous_driving_project/blob/master/readme/navigation.png" width="500">

#### 2.2.1 Run the Gazebo environment:

    roslaunch myhamster_navigation navigation.launch 

Send goal in rviz:

- Click '2D Nav Goal'.
- Click and drag at the position you want the robot to go.



### 2.3 Single Task Demo:

#### 2.3.1 Start simulation environment
    roslaunch myhamster_challenge start_gazebo.launch

#### 2.3.2 Run reverse parking node:
    roslaunch myhamster_challenge reverse_parking.launch

#### 2.3.3 Run hill climbing node:
    roslaunch myhamster_challenge up_hill.launch

#### 2.3.4 Run right angle turning node:
    roslaunch myhamster_challenge right_angle.launch

#### 2.3.5 Run side parking node:
    roslaunch myhamster_challenge side_parking.launch

#### 2.3.3 Run lane tracking node:
    roslaunch myhamster_challenge s_shape.launch


### 2.4 Run Whole System Demo:

#### 2.4.1 Start simulation environment
    roslaunch myhamster_challenge start_gazebo.launch

#### 2.4.1 Start control node
    roslaunch myhamster_challenge autorace_core.launch  
