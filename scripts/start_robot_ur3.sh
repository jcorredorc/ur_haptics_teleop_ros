#!/bin/sh
xterm  -e  " roslaunch ur_gazebo ur3_joint_limited.launch" &
sleep 3
xterm  -e  " roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch sim:=true limited:=true" & 
sleep 3
xterm  -e  " roslaunch ur3_moveit_config moveit_rviz.launch config:=true"




