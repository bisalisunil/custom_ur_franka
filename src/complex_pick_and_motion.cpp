#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

#define PI 3.14159265359

geometry_msgs::msg::Pose create_pose(double x, double y, double z, double roll, double pitch, double yaw)
{
  geometry_msgs::msg::Pose pose;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  pose.orientation = tf2::toMsg(q);
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  return pose;
}

void control_gripper(moveit::planning_interface::MoveGroupInterface& gripper_group, bool close)
{
  std::vector<std::string> joint_names = gripper_group.getJointNames();
  RCLCPP_INFO(rclcpp::get_logger("gripper_debug"), "Gripper has %zu joints", joint_names.size());

  for (const auto& name : joint_names)
    RCLCPP_INFO(rclcpp::get_logger("gripper_debug"), "Joint: %s", name.c_str());

  std::vector<double> joint_positions = gripper_group.getCurrentJointValues();

  if (joint_positions.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("gripper_debug"), "No joint values retrieved");
    return;
  }

  joint_positions[0] = close ? 0.8 : 0.0;

  gripper_group.setJointValueTarget(joint_positions);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (gripper_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    gripper_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("gripper_debug"), "Failed to plan gripper motion");
  }

  rclcpp::sleep_for(std::chrono::seconds(1));
}


void execute_trajectory(moveit::planning_interface::MoveGroupInterface& arm_group,
                        const std::vector<geometry_msgs::msg::Pose>& waypoints)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  arm_group.execute(trajectory);
}

std::vector<geometry_msgs::msg::Pose> generate_circular_trajectory(const geometry_msgs::msg::Pose& center, double radius, int steps)
{
  std::vector<geometry_msgs::msg::Pose> path;
  for (int i = 0; i <= steps; ++i)
  {
    double angle = 2 * PI * i / steps;
    geometry_msgs::msg::Pose p = center;
    p.position.x += radius * cos(angle);
    p.position.y += radius * sin(angle);
    path.push_back(p);
  }
  return path;
}

std::vector<geometry_msgs::msg::Pose> generate_lissajous_trajectory(const geometry_msgs::msg::Pose& center, double a, double b, int steps)
{
  std::vector<geometry_msgs::msg::Pose> path;
  for (int i = 0; i <= steps; ++i)
  {
    double t = 2 * PI * i / steps;
    geometry_msgs::msg::Pose p = center;
    p.position.x += a * sin(2 * t);
    p.position.y += b * sin(t);
    path.push_back(p);
  }
  return path;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto const node = rclcpp::Node::make_shared("ur5_pick_and_motion_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  RCLCPP_INFO(node->get_logger(), "Starting UR5 Pick and Motion Node");

  moveit::planning_interface::MoveGroupInterface arm_group(node, "ur5_manipulator");
  moveit::planning_interface::MoveGroupInterface gripper_group(node, "Robotiq_gripper");

  // Set end effector link (adjust the link name based on your setup)
  arm_group.setEndEffectorLink("robotiq_85_base_link");
  RCLCPP_INFO(node->get_logger(), "End effector link set to: %s", arm_group.getEndEffectorLink().c_str());

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Set safe velocity/acceleration
  arm_group.setMaxVelocityScalingFactor(0.3);
  arm_group.setMaxAccelerationScalingFactor(0.3);
  control_gripper(gripper_group, false);
  // Move to home position
  std::vector<double> home_joints = {0, -PI / 2, 0, -PI / 2, 0, 0};
  arm_group.setJointValueTarget(home_joints);
  arm_group.move();
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(node->get_logger(), "Going to pickking position");

  std::vector<double> pick_joints = {0.0, -PI / 4, PI / 3, 0, 0, 0};
  arm_group.setJointValueTarget(pick_joints);
  arm_group.move();
  rclcpp::sleep_for(std::chrono::seconds(1));

  gripper_group.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.8);  // open = ~0.8, close = ~0.0
  gripper_group.move();


  std::vector<double> place_joints = {PI, -PI / 4, PI / 4, 2 * PI / 3, 3 * PI / 2, 3 * PI / 2};
  arm_group.setJointValueTarget(place_joints);
  arm_group.move();
  rclcpp::sleep_for(std::chrono::seconds(1));
    // // Pick sequence
  // geometry_msgs::msg::Pose pre_grasp = create_pose(0.0, -PI / 2, PI / 3, 0, 0, 0);
  // geometry_msgs::msg::Pose grasp = pre_grasp;
  // grasp.position.z -= 0.08;

  // execute_trajectory(arm_group, {pre_grasp});
  // execute_trajectory(arm_group, {grasp});
  // control_gripper(gripper_group, true);  // Close gripper
  // execute_trajectory(arm_group, {pre_grasp});

  // // Drop and complex motion
  // geometry_msgs::msg::Pose drop_pre = create_pose(0.1, -0.3, 0.3, PI, 0, 0);
  // arm_group.setPoseTarget(drop_pre);
  // arm_group.move();
  // rclcpp::sleep_for(std::chrono::seconds(1));

  // // Circle + Lissajous motions
  // auto circular_path = generate_circular_trajectory(drop_pre, 0.05, 30);
  // execute_trajectory(arm_group, circular_path);

  // auto lissajous_path = generate_lissajous_trajectory(drop_pre, 0.05, 0.05, 40);
  // execute_trajectory(arm_group, lissajous_path);

  // // Drop and return home
  // control_gripper(gripper_group, false);
  // rclcpp::sleep_for(std::chrono::seconds(1));
  // arm_group.setJointValueTarget(home_joints);
  // arm_group.move();

  RCLCPP_INFO(node->get_logger(), "Completed pick and motion sequence");


  rclcpp::shutdown();
  return 0;
}
