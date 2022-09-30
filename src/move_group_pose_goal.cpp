#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #include "ros/ros.h"
// #include "std_msgs/String.h"
// #include <sstream>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangeably.
  ////static const std::string PLANNING_GROUP = "panda_arm";
  static const std::string PLANNING_GROUP = "manipulator";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  // moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  // moveit_visual_tools::MoveItVisualTools visual_tools("worle_group_interface.setPlanningTime(10.0)d");

  // moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node,
  // "base_link","move_group_tutorial",move_group.getRobotModel());
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ","));

  // We can get the current pose of the end effector position and orientation
  geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
  ROS_INFO("x:%f y:%f z:%f ", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo. Planning to a Pose goal");

  //   .. _move_group_interface-planning-to-pose-goal:

  //   Planning to a Pose goal
  //   ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  tf2::Quaternion myQuaternion;
  geometry_msgs::Quaternion quat_msg;
  myQuaternion.setRPY(0, 0, tau / 4);
  quat_msg = tf2::toMsg(myQuaternion);

  geometry_msgs::Pose target_pose1;
  //   target_pose1.orientation.w = 1.0;
  //   target_pose1.position.x = 0.28;
  //   target_pose1.position.y = -0.2;
  //   target_pose1.position.z = 0.5;
  // target_pose1.orientation.w = 1.0;
  target_pose1.orientation = quat_msg;
  target_pose1.position.x = 0.2;
  target_pose1.position.y = 0.15;
  target_pose1.position.z = 0.6;
  move_group_interface.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group_interface
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1", rvt::MEDIUM);
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Finally, to execute the trajectory stored in my_plan, you could use the following method call:
  // Note that this can lead to problems if the robot moved in the meanwhile.
  move_group_interface.execute(my_plan);

  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // If you do not want to inspect the planned trajectory,
  // the following is a more robust combination of the two-step plan+execute pattern shown above
  // and should be preferred. Note that the pose goal we had set earlier is still active,
  // so the robot will try to move to that goal.

  // move_group_interface.move();

  //   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue Planning join-space goal");
  current_pose = move_group_interface.getCurrentPose();
  ROS_INFO("x:%f y:%f z:%f ", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);

  ros::shutdown();
  return 0;
}