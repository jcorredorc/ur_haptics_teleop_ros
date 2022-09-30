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

  //   moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "base_link", "move_group_tutorial",
  //                                                       move_group.getRobotModel());
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

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group_interface
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success;  //= (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  //==================================================
  //   Planning to the Up Pose goal
  //   ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  tf2::Quaternion myQuaternion;
  geometry_msgs::Quaternion quat_msg;
  myQuaternion.setRPY(0, 0, tau / 4);
  quat_msg = tf2::toMsg(myQuaternion);
  // UP x:0.000090 y:0.194250 z:0.694150
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = quat_msg;
  target_pose1.position.x = 0.000090;
  target_pose1.position.y = 0.194250;
  target_pose1.position.z = 0.694150;
  move_group_interface.setPoseTarget(target_pose1);

  // // Now, we call the planner to compute the plan and visualize it.
  // // Note that we are just planning, not asking move_group_interface
  // // to actually move the robot.
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  move_group_interface.execute(my_plan);

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo, start pose");

  //=============================================================================

  // quat_msg = tf2::toMsg(myQuaternion);
  // target_pose1.orientation = quat_msg;
  // target_pose1.position.x = 0.228337;
  // target_pose1.position.y = 0.407855;
  // target_pose1.position.z = 0.435218;
  // move_group_interface.setPoseTarget(target_pose1);
  // success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose start) %s", success ? "" : "FAILED");
  // move_group_interface.execute(my_plan);
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // ................................................................

  // UP x:0.363315 y:-0.186520 z:0.517588
  moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
  geometry_msgs::Pose start_pose2;
  // start_pose2.orientation.w = 1.0;
  myQuaternion.setRPY(0, 0, 0);
  quat_msg = tf2::toMsg(myQuaternion);
  start_pose2.orientation = quat_msg;
  start_pose2.position.x = 0.363315;
  start_pose2.position.y = -0.186520;  // 0.194331;
  start_pose2.position.z = 0.517588;   // 0.1;  // 0.0064503;
  // start_state.setFromIK(joint_model_group, start_pose2);
  // start_state.setFromIK(joint_model_group, target_pose1);
  // move_group_interface.setStartState(start_state);

  // Now we will plan to the earlier pose target from the new
  // start state that we have just created.
  move_group_interface.setPoseTarget(start_pose2);

  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  move_group_interface.execute(my_plan);

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo, Cartesian Path");

  // .........................................................................

  // -------
  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a Cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note that we are starting
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list but adding it can help with visualizations
  std::vector<geometry_msgs::Pose> waypoints;
  // waypoints.push_back(target_pose1);
  waypoints.push_back(start_pose2);
  // waypoints.push_back(target_pose1);

  geometry_msgs::Pose target_pose3 = start_pose2;

  double offset = 0.15;
  target_pose3.position.z -= offset;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.y += offset;
  waypoints.push_back(target_pose3);  // right

  target_pose3.position.z += offset;
  target_pose3.position.y -= offset;
  // target_pose3.position.x -= offset;
  waypoints.push_back(target_pose3);  // up and left

  // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in Cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  // const double eef_step = 0.1;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();

  // Cartesian motions should often be slow, e.g. when approaching objects. The speed of cartesian
  // plans cannot currently be set through the maxVelocityScalingFactor, but requires you to time
  // the trajectory manually, as described `here
  // <https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4>`_.
  // Pull requests are welcome.
  //
  // You can execute a trajectory like this.
  move_group_interface.execute(trajectory);

  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo, Adding Objects");

  // -------

  ros::shutdown();
  return 0;
}