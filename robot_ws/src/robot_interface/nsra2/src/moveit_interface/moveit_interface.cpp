/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Noa Sendlhofer
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Noa Sendlhofer

#include "moveit_interface.h"

MoveItInterface::MoveItInterface(std::shared_ptr<rclcpp::Node> node)
{
    this->node = node;

    static const std::string NSRA_PLANNING_GROUP = "nsra";
    static const std::string HAND_PLANNING_GROUP = "hand";
    this->nsra_move_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node, NSRA_PLANNING_GROUP);
    this->hand_move_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node, HAND_PLANNING_GROUP);
    this->joint_model_group = nsra_move_group->getCurrentState()->getJointModelGroup(NSRA_PLANNING_GROUP);
    this->hand_joint_model_group = hand_move_group->getCurrentState()->getJointModelGroup(HAND_PLANNING_GROUP);
    this->visual_tools = std::make_unique<moveit_visual_tools::MoveItVisualTools>(node, "base_link", "nsra_planning", nsra_move_group->getRobotModel());

    this->visual_tools->deleteAllMarkers();
    this->visual_tools->loadRemoteControl();

    _placeTable();
    nsra_move_group->setMaxVelocityScalingFactor(1.0);
    nsra_move_group->setMaxAccelerationScalingFactor(1.0);

    this->update_scene = true;
}

void MoveItInterface::updateScene(std::vector<Bottle> bottles)
{
    if (!this->update_scene) { return; }

    std::vector<moveit_msgs::msg::CollisionObject> new_objects;

    moveit_msgs::msg::CollisionObject object_to_attach;
    object_to_attach.header.frame_id = nsra_move_group->getPlanningFrame();
    object_to_attach.id = std::to_string(bottles[0].id);
    new_objects.push_back(object_to_attach);

    shape_msgs::msg::SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = cylinder_primitive.CYLINDER;
    cylinder_primitive.dimensions.resize(2);
    cylinder_primitive.dimensions[cylinder_primitive.CYLINDER_HEIGHT] = 0.22;
    cylinder_primitive.dimensions[cylinder_primitive.CYLINDER_RADIUS] = 0.03;

    geometry_msgs::msg::Pose grab_pose;
    grab_pose.orientation.w = 1.0;
    grab_pose.position.x = bottles[0].coord_3d.x() / 1000.0;
    grab_pose.position.y = bottles[0].coord_3d.y() / 1000.0;
    grab_pose.position.z = bottles[0].coord_3d.z() / 1000.0 - 0.11;

    object_to_attach.primitives.push_back(cylinder_primitive);
    object_to_attach.primitive_poses.push_back(grab_pose);
    object_to_attach.operation = object_to_attach.ADD;

    this->objects = new_objects;
    planning_scene_interface.applyCollisionObject(object_to_attach);
    RCLCPP_INFO(this->node->get_logger(), "Scene updated");
}

void MoveItInterface::graspObject(int id)
{
    visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    this->update_scene = false;
    moveit_msgs::msg::CollisionObject object;

    bool found = false;
    for (auto & bottle : this->objects)
    {
        if (bottle.id == std::to_string(id))
        {
            object = bottle;
            found = true;
        }
    }

    if (!found) {
        RCLCPP_ERROR(this->node->get_logger(), "Given Object ID not found");
        return;
    }

    _grasp(object);
}

bool MoveItInterface::_grasp(moveit_msgs::msg::CollisionObject object)
{
    _openGripper();
    if (_pick(object))
    {
        RCLCPP_WARN(this->node->get_logger(), "Object pick failed!");
        return false;
    }
    this->update_scene = true;
    return true;
}

bool MoveItInterface::_pick(moveit_msgs::msg::CollisionObject object)
{
    std::vector<std::string> bottle_ids = {object.id};
    geometry_msgs::msg::Pose object_pos = planning_scene_interface.getObjectPoses(bottle_ids)[0];

    std::cout << object_pos.position.x << std::endl;

    tf2::Quaternion orientation;
    orientation.setRPY(M_PI / 2, 0, 0);

    //pre_grasp
    geometry_msgs::msg::Pose pre_grasp_pose = object_pos;
    this->nsra_move_group->setStartState(*this->nsra_move_group->getCurrentState());
    pre_grasp_pose.orientation = __toMsg(orientation);
    pre_grasp_pose.position.x = object_pos.position.x - 0.3;
    this->nsra_move_group->setPoseTarget(pre_grasp_pose);
    moveit::planning_interface::MoveGroupInterface::Plan pre_grasp_plan;
    bool success = (this->nsra_move_group->plan(pre_grasp_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) { return false; }
    this->nsra_move_group->move();
    RCLCPP_INFO(this->node->get_logger(), "Grasp!");
    //grasp
    this->nsra_move_group->setStartState(*this->nsra_move_group->getCurrentState());
    geometry_msgs::msg::Pose grasp_pose = object_pos;
    grasp_pose.orientation = __toMsg(orientation);
    grasp_pose.position.x = object_pos.position.x - 0.16;
    this->nsra_move_group->setPoseTarget(grasp_pose);
    moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
    success = (this->nsra_move_group->plan(grasp_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) { return false; }
    this->nsra_move_group->move();

    _closeGripper();
    RCLCPP_INFO(this->node->get_logger(), "Post Grasp!");
    //post_grasp
    this->nsra_move_group->setStartState(*this->nsra_move_group->getCurrentState());
    geometry_msgs::msg::Pose post_grasp = object_pos;
    post_grasp.orientation = __toMsg(orientation);
    post_grasp.position.x = object_pos.position.x - 0.16;
    post_grasp.position.z = object_pos.position.z + 0.2;
    this->nsra_move_group->setStartStateToCurrentState();
    this->nsra_move_group->setPoseTarget(post_grasp);
    moveit::planning_interface::MoveGroupInterface::Plan post_grasp_plan;
    success = (this->nsra_move_group->plan(post_grasp_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) { return false; }
    this->nsra_move_group->move();

    return true;
}

void MoveItInterface::_openGripper()
{
    moveit::core::RobotStatePtr current_state = hand_move_group->getCurrentState(10);
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(hand_joint_model_group, joint_group_positions);

    joint_group_positions[0] = 0.0;
    joint_group_positions[1] = 0.0;
    hand_move_group->setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    if (hand_move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        hand_move_group->move();
    }
}

void MoveItInterface::_closeGripper()
{
    moveit::core::RobotStatePtr current_state = hand_move_group->getCurrentState(10);
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(hand_joint_model_group, joint_group_positions);

    joint_group_positions[0] = 0.02;
    joint_group_positions[1] = 0.02;
    hand_move_group->setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    if (hand_move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        hand_move_group->move();
    }
}

void MoveItInterface::_placeTable()
{
    moveit_msgs::msg::CollisionObject table_object;
    table_object.header.frame_id = nsra_move_group->getPlanningFrame();
    table_object.id = "table";

    shape_msgs::msg::SolidPrimitive box_primitive;
    box_primitive.type = box_primitive.BOX;
    box_primitive.dimensions.resize(3);
    box_primitive.dimensions[box_primitive.BOX_X] = 0.8;
    box_primitive.dimensions[box_primitive.BOX_Y] = 0.8;
    box_primitive.dimensions[box_primitive.BOX_Z] = 0.1;

    geometry_msgs::msg::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 0.4;
    table_pose.position.y = 0.27;
    table_pose.position.z = - 0.05;

    table_object.primitives.push_back(box_primitive);
    table_object.primitive_poses.push_back(table_pose);
    table_object.operation = table_object.ADD;

    planning_scene_interface.applyCollisionObject(table_object);
    RCLCPP_INFO(this->node->get_logger(), "Table placed");
}

geometry_msgs::msg::Quaternion MoveItInterface::__toMsg(tf2::Quaternion tf2_quat)
{
    geometry_msgs::msg::Quaternion geom_quat;
    geom_quat.w = tf2_quat.w();
    geom_quat.x = tf2_quat.x();
    geom_quat.y = tf2_quat.y();
    geom_quat.z = tf2_quat.z();
    return geom_quat;
}
