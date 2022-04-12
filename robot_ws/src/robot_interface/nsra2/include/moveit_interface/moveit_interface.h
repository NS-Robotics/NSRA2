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

#ifndef ROBOT_INTERFACE_MOVEIT_INTERFACE_H_
#define ROBOT_INTERFACE_MOVEIT_INTERFACE_H_

#include "rclcpp/rclcpp.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;

struct Bottle
{
    Eigen::Vector3d coord_3d;
    Eigen::Vector2f left_coord_2d;
    Eigen::Vector2f right_coord_2d;
    int id;
};

class MoveItInterface
{
public:
    explicit MoveItInterface(std::shared_ptr<rclcpp::Node> node);
    void updateScene(std::vector<Bottle> bottles);
    void graspObject(int id);

private:
    void _placeTable();
    bool _grasp(const moveit_msgs::msg::CollisionObject& object);
    void _openGripper();
    void _closeGripper();
    bool _pick(const moveit_msgs::msg::CollisionObject& object);
    bool _place(const moveit_msgs::msg::CollisionObject& object);
    void _calculateOrientation(geometry_msgs::msg::Pose &object_pos, double approach);
    static geometry_msgs::msg::Quaternion _toMsg(tf2::Quaternion tf2_quat);

    std::shared_ptr<rclcpp::Node> node;

    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> nsra_move_group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup* joint_model_group;

    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> hand_move_group;
    const moveit::core::JointModelGroup* hand_joint_model_group;

    std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools;

    std::vector<moveit_msgs::msg::CollisionObject> objects;
    geometry_msgs::msg::Pose place_pos;
    bool object_attached = false;

    bool update_scene;
};

#endif //ROBOT_INTERFACE_MOVEIT_INTERFACE_H_
