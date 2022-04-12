#include "rclcpp/rclcpp.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("nsra2_interface");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("nsra2_interface", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]()
                { executor.spin(); }).detach();


    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "NSRA_Demo", rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    geometry_msgs::msg::Pose target_pose1;
    target_pose1.orientation.w = 0.0;
    target_pose1.position.x = 0.2;
    target_pose1.position.y = 0.2;
    target_pose1.position.z = 0.2;
    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    move_group.move();

    RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    RCLCPP_INFO(LOGGER, "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[0] = -1.0;  // radians
    move_group.setJointValueTarget(joint_group_positions);

    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");




}