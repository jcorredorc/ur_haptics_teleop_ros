#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
  ROS_INFO("UP x:%f y:%f z:%f ", current_pose.pose.position.x, current_pose.pose.position.y,
           current_pose.pose.position.z);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group_interface
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success;  // = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

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

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  //======================================================

  // move to start point (izquierda) to goal (derecha)
  // izquierda x:0.228337 y:0.407855 z:0.435218
  // derecha  x:-0.223485 y:0.401569 z:0.449578

  // The following orientations doesn't fount a valid solution: yaw: 0, tau/6,
  myQuaternion.setRPY(0, 0, tau / 4);  // test with valid Yaw: tau/4; tau/5
  quat_msg = tf2::toMsg(myQuaternion);
  target_pose1.orientation = quat_msg;
  target_pose1.position.x = 0.228337;
  target_pose1.position.y = 0.407855;
  target_pose1.position.z = 0.435218;
  move_group_interface.setPoseTarget(target_pose1);
  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose start) %s", success ? "" : "FAILED");
  move_group_interface.execute(my_plan);
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // -------
  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a pose goal for our group.
  // First define the path constraint.
  moveit_msgs::OrientationConstraint ocm;
  // ocm.link_name = "panda_link7";
  // ocm.header.frame_id = "panda_link0";
  // ocm.link_name = "forearm_link"; // upper_arm_link, shoulder_link
  ocm.link_name = "ee_link";  //
  ocm.header.frame_id = "base_link";
  // myQuaternion.setRPY(0, 0, tau / 4);
  // quat_msg = tf2::toMsg(myQuaternion);
  ocm.orientation = quat_msg;
  // ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;
  // ocm.weight = 0.4;  // near zero less priority

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group_interface.setPathConstraints(test_constraints);

  // derecha  x:-0.223485 y:0.401569 z:0.449578

  moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
  geometry_msgs::Pose start_pose2;
  // start_pose2.orientation.w = 1.0;
  start_pose2.orientation = quat_msg;
  start_pose2.position.x = -0.223485;
  start_pose2.position.y = 0.401569;  // 0.194331;
  start_pose2.position.z = 0.449578;  // 0.1;  // 0.0064503;
  // start_state.setFromIK(joint_model_group, start_pose2);
  start_state.setFromIK(joint_model_group, target_pose1);
  move_group_interface.setStartState(start_state);

  // Now we will plan to the earlier pose target from the new
  // start state that we have just created.
  move_group_interface.setPoseTarget(start_pose2);
  //---------------

  // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to
  move_group_interface.setPlanningTime(10.0);

  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  // visual_tools.prompt("next step, Cartesian Path");

  // move_group_interface.move();

  // Finally, to execute the trajectory stored in my_plan, you could use the following method call:
  // Note that this can lead to problems if the robot moved in the meanwhile.
  move_group_interface.execute(my_plan);

  // When done with the path constraint be sure to clear it.
  move_group_interface.clearPathConstraints();

  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo, Cartesian Path");

  // -------

  ros::shutdown();
  return 0;
}