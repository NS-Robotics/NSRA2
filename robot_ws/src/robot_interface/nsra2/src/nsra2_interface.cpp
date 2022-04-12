#include "rclcpp/rclcpp.hpp"

#include "moveit_interface.h"
#include "message_handler.h"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("nsra2_interface");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("nsra2_interface", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread executor_thread([&executor]()
                                { executor.spin(); });

    auto moveit_interface = std::make_shared<MoveItInterface>(move_group_node);
    MessageHandler message_handler(move_group_node, moveit_interface);

    moveit_interface->graspObject(0);

    executor_thread.join();

    /*
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
    nsra_move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (nsra_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    nsra_move_group.move();

    RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    RCLCPP_INFO(LOGGER, "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    moveit::core::RobotStatePtr current_state = nsra_move_group.getCurrentState(10);

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[0] = -1.0;  // radians
    nsra_move_group.setJointValueTarget(joint_group_positions);

    nsra_move_group.setMaxVelocityScalingFactor(0.05);
    nsra_move_group.setMaxAccelerationScalingFactor(0.05);

    success = (nsra_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    */
}