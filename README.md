# ur_haptics_teleop_ros

## Install

### Requirements

* Novint Falcon
* Ubuntu 2004
* ROS Noetic

## Install the Falcon

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
or 

```
git clone -b melodic-devel https://github.com/ros-industrial/universal_robot
```


```
catkin build
```

Test the UR3 robot 

```
roslaunch ur_gazebo ur3_bringup.launch
```

3. Test UR3 with Moveit!


```
<!-- roslaunch ur_gazebo ur3_joint_limited.launch -->
roslaunch ur_gazebo ur3_bringup.launch
roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch sim:=true limited:=true
roslaunch ur3_moveit_config moveit_rviz.launch config:=true
```

See the [moveit doc](https://ros-planning.github.io/moveit_tutorials/) and [UR - ros industrial](https://github.com/ros-industrial/universal_robot)


4. Clone this repo and build it

```
git clone https://github.com/jcorredorc/ur_haptics_teleop_ros.git
```


## Usage

1. Start the robot simulation, moveit, Gazebo and Rviz

```
rosrun ur_haptics_teleop_ros start_robot_ur3.sh
```


2. 

Test a simple pick and place task (see [Edx course](https://www.edx.org/es/course/hello-real-world-with-ros-robot-operating-system))

```
rosrun ur_haptics_teleop_ros pick_place_test.py
```


3. 

Test the move_group interface tutorial in the UR3.

<!-- ```
rosrun ur_haptics_teleop_ros move_group_interface_tutorial
``` -->

In Rviz, include in Panels/Add New panel, RvizVisualToolsGui, to navigate throughout the tutorial. Also include the Marker display and configure the Marker topic to /rviz_visual_tools


```
rosrun ur_haptics_teleop_ros move_group_pose_goal_node
```

```
rosrun ur_haptics_teleop_ros move_group_joint_goal_node
```

```
rosrun ur_haptics_teleop_ros move_group_path_constrain_node
```

```
rosrun ur_haptics_teleop_ros move_group_cartesian_path_node
```

```
rosrun ur_haptics_teleop_ros move_group_adding_objects_node
```

4.  moveit_servo examples

see the [github site](https://github.com/ros-planning/moveit/tree/master/moveit_ros/moveit_servo)


```
rosrun ur_haptics_teleop_ros start_robot_ur3.sh
```

In RViz, grab the red/blue/green “interactive marker” and drag the robot to a non-singular position (not all zero joint angles) that is not close to a joint limit. Click “plan and execute” to move the robot to that pose.

```
rosservice call /controller_manager/switch_controller "start_controllers: ['joint_group_position_controller']
stop_controllers: ['arm_controller']
strictness: 0
start_asap: false
timeout: 0.0"
```

```
roslaunch ur_haptics_teleop_ros cpp_interface_example.launch
```

Usign URSim and moveit, get this error

```
[ERROR] [1680131092.166186633]: Link 'ee_link' not found in model 'ur3_robot'
[servo_server-1] process has died [pid 149503, exit code -11, cmd /home/javier/catkin_ur/devel/lib/ur_haptics_teleop_ros/cpp_interface_example_node __name:=servo_server __log:=/home/javier/.ros/log/08d99332-ce81-11ed-a66a-a394436df276/servo_server-1.log].
log file: /home/javier/.ros/log/08d99332-ce81-11ed-a66a-a394436df276/servo_server-1*.log
```


