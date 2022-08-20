# ur_haptics_teleop_ros

## Install

### Requirements

* Novint Falcon
* Ubuntu 2004
* ROS Noetic

### Install the Falcon

See how to install and test the Falcon in ROS [here](doc/install_falcon.md)

## Install UR3 Gazebo

1. Install Moveit!

```
sudo apt install ros-noetic-moveit
```

2. Install UR package 

Follow the instructions [here](https://github.com/ros-industrial/universal_robot.git) using this repo,

```
git clone --depth 1 --branch melodic-devel https://github.com/ros-industrial/universal_robot.git
```

```
catkin build
```

Test the robot UR3

```
roslaunch ur_gazebo ur3.launch
```

3. Test UR3 with Moveit!


```
roslaunch ur_gazebo ur3_joint_limited.launch
roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch sim:=true limited:=true
roslaunch ur3_moveit_config moveit_rviz.launch config:=true
```

see the [moveit doc](https://ros-planning.github.io/moveit_tutorials/) 

## Usage







