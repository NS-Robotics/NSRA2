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

    nsra_move_group->setMaxVelocityScalingFactor(1.0);
    nsra_move_group->setMaxAccelerationScalingFactor(1.0);

    this->place_pos.position.x = 0.4;
    this->place_pos.position.y = 0.5;
    this->place_pos.position.z = - 0.12;

    this->update_scene = true;

    std::vector<std::string> known_objects = this->planning_scene_interface.getKnownObjectNames();
    this->planning_scene_interface.removeCollisionObjects(known_objects);
    _placeTable();
}

void MoveItInterface::updateScene(std::vector<Bottle> bottles)
{
    if (!this->update_scene) { return; }

    std::vector<moveit_msgs::msg::CollisionObject> new_objects;

    if (bottles.size() != this->objects.size())
    {
        std::vector<std::string> known_objects = this->planning_scene_interface.getKnownObjectNames();
        this->planning_scene_interface.removeCollisionObjects(known_objects);
        _placeTable();
    }

    for (auto & bottle : bottles)
    {
        moveit_msgs::msg::CollisionObject object_to_attach;
        object_to_attach.header.frame_id = nsra_move_group->getPlanningFrame();
        object_to_attach.id = std::to_string(bottle.id);

        shape_msgs::msg::SolidPrimitive cylinder_primitive;
        cylinder_primitive.type = cylinder_primitive.CYLINDER;
        cylinder_primitive.dimensions.resize(2);
        cylinder_primitive.dimensions[cylinder_primitive.CYLINDER_HEIGHT] = 0.22;
        cylinder_primitive.dimensions[cylinder_primitive.CYLINDER_RADIUS] = 0.03;

        geometry_msgs::msg::Pose grab_pose;
        grab_pose.orientation.w = 1.0;
        grab_pose.position.x = bottle.coord_3d.x() / 1000.0;
        grab_pose.position.y = bottle.coord_3d.y() / 1000.0;
        grab_pose.position.z = bottle.coord_3d.z() / 1000.0 - 0.10;

        object_to_attach.primitives.push_back(cylinder_primitive);
        object_to_attach.primitive_poses.push_back(grab_pose);
        object_to_attach.operation = object_to_attach.ADD;

        new_objects.push_back(object_to_attach);
    }

    this->objects = new_objects;
    planning_scene_interface.applyCollisionObjects(new_objects);
    RCLCPP_INFO(this->node->get_logger(), "Scene updated");
}

void MoveItInterface::graspObject(int id)
{
    while (true)
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

        if (_grasp(object))
        {
            RCLCPP_INFO(this->node->get_logger(), "Object grasp successfull!");
        }
    }
}

bool MoveItInterface::_grasp(const moveit_msgs::msg::CollisionObject& object)
{
    _openGripper();
    if (!_pick(object))
    {
        RCLCPP_ERROR(this->node->get_logger(), "Object pick failed!");
        if (this->object_attached)
        {
            this->hand_move_group->detachObject(object.id);
            this->object_attached = false;
            _openGripper();
        }
        this->update_scene = true;
        return false;
    }

    if (!_place(object))
    {
        RCLCPP_ERROR(this->node->get_logger(), "Object place failed!");
        if (this->object_attached)
        {
            this->hand_move_group->detachObject(object.id);
            this->object_attached = false;
            _openGripper();
        }
        this->update_scene = true;
        return false;
    }

    this->update_scene = true;
    return true;
}

bool MoveItInterface::_pick(const moveit_msgs::msg::CollisionObject& object)
{
    std::vector<std::string> bottle_ids = {object.id};
    auto object_map = planning_scene_interface.getObjectPoses(bottle_ids);
    geometry_msgs::msg::Pose object_pos = object_map.find(object.id)->second;

    this->nsra_move_group->setSupportSurfaceName("table");

    //pre_grasp
    geometry_msgs::msg::Pose pre_grasp_pose = object_pos;
    this->nsra_move_group->setStartState(*this->nsra_move_group->getCurrentState());
    _calculateOrientation(pre_grasp_pose, 0.25);
    pre_grasp_pose.position.z = object_pos.position.z + 0.2;
    this->nsra_move_group->setPoseTarget(pre_grasp_pose);
    moveit::planning_interface::MoveGroupInterface::Plan pre_grasp_plan;
    if (this->nsra_move_group->plan(pre_grasp_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) { return false; }
    if (this->nsra_move_group->move() != moveit::planning_interface::MoveItErrorCode::SUCCESS) { return false; }

    //grasp
    this->nsra_move_group->setStartState(*this->nsra_move_group->getCurrentState());
    geometry_msgs::msg::Pose grasp_pose = object_pos;
    _calculateOrientation(grasp_pose, 0.16);
    grasp_pose.position.z = object_pos.position.z + 0.01;
    this->nsra_move_group->setPoseTarget(grasp_pose);
    moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
    if (this->nsra_move_group->plan(grasp_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) { return false; }
    if (this->nsra_move_group->move() != moveit::planning_interface::MoveItErrorCode::SUCCESS) { return false; }

    _closeGripper();
    std::vector<std::string> touch_links;
    touch_links.push_back("Component7_1");
    touch_links.push_back("Component8_1");
    this->hand_move_group->attachObject(object.id, "Component7_1", touch_links);
    this->object_attached = true;

    //post_grasp_pose
    this->nsra_move_group->setStartState(*this->nsra_move_group->getCurrentState());
    geometry_msgs::msg::Pose post_grasp_pose = object_pos;
    _calculateOrientation(post_grasp_pose, 0.16);
    post_grasp_pose.position.z = object_pos.position.z + 0.2;
    this->nsra_move_group->setStartStateToCurrentState();
    this->nsra_move_group->setPoseTarget(post_grasp_pose);
    moveit::planning_interface::MoveGroupInterface::Plan post_grasp_plan;
    if (this->nsra_move_group->plan(post_grasp_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) { return false; }
    if (this->nsra_move_group->move() != moveit::planning_interface::MoveItErrorCode::SUCCESS) { return false; }

    return true;
}

bool MoveItInterface::_place(const moveit_msgs::msg::CollisionObject& object)
{
    std::vector<std::string> bottle_ids = {object.id};
    auto object_map = planning_scene_interface.getObjectPoses(bottle_ids);
    geometry_msgs::msg::Pose object_pos = object_map.find(object.id)->second;

    this->nsra_move_group->setSupportSurfaceName("table");

    //place
    geometry_msgs::msg::Pose place_pose = this->place_pos;
    this->nsra_move_group->setStartState(*this->nsra_move_group->getCurrentState());
    place_pose.position.z = this->place_pos.position.z + 0.05;
    _calculateOrientation(place_pose, 0.0);
    this->nsra_move_group->setPoseTarget(place_pose);
    moveit::planning_interface::MoveGroupInterface::Plan place_plan;
    if (this->nsra_move_group->plan(place_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) { return false; }
    if (this->nsra_move_group->move() != moveit::planning_interface::MoveItErrorCode::SUCCESS) { return false; }

    this->hand_move_group->detachObject(object.id);
    this->object_attached = false;
    _openGripper();

    //post_place
    geometry_msgs::msg::Pose post_place_pose = this->place_pos;
    this->nsra_move_group->setStartState(*this->nsra_move_group->getCurrentState());
    _calculateOrientation(post_place_pose, 0.16);
    post_place_pose.position.z = this->place_pos.position.z + 0.2;
    this->nsra_move_group->setPoseTarget(post_place_pose);
    moveit::planning_interface::MoveGroupInterface::Plan post_place_plan;
    if (this->nsra_move_group->plan(post_place_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) { return false; }
    if (this->nsra_move_group->move() != moveit::planning_interface::MoveItErrorCode::SUCCESS) { return false; }

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
    table_pose.position.z = - 0.28;

    table_object.primitives.push_back(box_primitive);
    table_object.primitive_poses.push_back(table_pose);
    table_object.operation = table_object.ADD;

    planning_scene_interface.applyCollisionObject(table_object);
    RCLCPP_INFO(this->node->get_logger(), "Table placed");
}

geometry_msgs::msg::Quaternion MoveItInterface::_toMsg(tf2::Quaternion tf2_quat)
{
    geometry_msgs::msg::Quaternion geom_quat;
    geom_quat.w = tf2_quat.w();
    geom_quat.x = tf2_quat.x();
    geom_quat.y = tf2_quat.y();
    geom_quat.z = tf2_quat.z();
    return geom_quat;
}

void MoveItInterface::_calculateOrientation(geometry_msgs::msg::Pose &object_pos, double approach)
{
    tf2::Quaternion orientation;

    if (object_pos.position.x < 0 && object_pos.position.y > 0)
    {
        orientation.setRPY(M_PI / 2, 0, M_PI / 2);
        object_pos.position.y = object_pos.position.y - approach;
    }
    else if (object_pos.position.x > 0 && object_pos.position.y > 0.4)
    {
        orientation.setRPY(M_PI / 2, 0, M_PI / 4);
        object_pos.position.y = object_pos.position.y - sqrt(approach*approach/2);
        object_pos.position.x = object_pos.position.x - sqrt(approach*approach/2);
    }
    else if (object_pos.position.x > 0 && object_pos.position.y > 0.14)
    {
        orientation.setRPY(M_PI / 2, 0, 0);
        object_pos.position.x = object_pos.position.x - approach;
    }
    else if (object_pos.position.x > 0 && object_pos.position.y < 0.14)
    {
        orientation.setRPY(M_PI / 2, 0, - M_PI / 4);
        object_pos.position.y = object_pos.position.y + sqrt(approach*approach/2);
        object_pos.position.x = object_pos.position.x - sqrt(approach*approach/2);
    }
    else if (object_pos.position.x < 0 && object_pos.position.y < 0)
    {
        orientation.setRPY(M_PI / 2, 0, - M_PI / 2);
        object_pos.position.y = object_pos.position.y + approach;
    }

    object_pos.orientation = _toMsg(orientation);
}
