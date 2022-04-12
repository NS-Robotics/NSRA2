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

    static const std::string PLANNING_GROUP = "nsra";
    this->move_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node, PLANNING_GROUP);
    this->joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    this->visual_tools = std::make_unique<moveit_visual_tools::MoveItVisualTools>(node, "base_link", "nsra_planning", move_group->getRobotModel());

    this->visual_tools->deleteAllMarkers();
    this->visual_tools->loadRemoteControl();
}

void MoveItInterface::updateScene(std::vector<Bottle> bottles)
{
    planning_scene_interface.removeCollisionObjects(this->object_ids);

    std::vector<std::string> object_ids;

    moveit_msgs::msg::CollisionObject object_to_attach;
    object_to_attach.id = std::to_string(bottles[0].id);
    object_ids.push_back(std::to_string(bottles[0].id));

    shape_msgs::msg::SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = cylinder_primitive.CYLINDER;
    cylinder_primitive.dimensions.resize(2);
    cylinder_primitive.dimensions[cylinder_primitive.CYLINDER_HEIGHT] = 0.20;
    cylinder_primitive.dimensions[cylinder_primitive.CYLINDER_RADIUS] = 0.04;

    geometry_msgs::msg::Pose grab_pose;
    grab_pose.orientation.w = 1.0;
    grab_pose.position.x = bottles[0].coord_3d.x() / 1000.0;
    grab_pose.position.y = bottles[0].coord_3d.y() / 1000.0;
    grab_pose.position.z = bottles[0].coord_3d.z() / 1000.0;

    object_to_attach.primitives.push_back(cylinder_primitive);
    object_to_attach.primitive_poses.push_back(grab_pose);
    object_to_attach.operation = object_to_attach.ADD;

    this->object_ids = object_ids;
    planning_scene_interface.applyCollisionObject(object_to_attach);
}