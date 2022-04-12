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

#include "message_handler.h"

MessageHandler::MessageHandler(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<MoveItInterface> moveit_interface)
{
    this->node = node;
    this->moveit_interface = moveit_interface;

    this->bottle_subscriber = this->node->create_subscription<nssc_interface::msg::ObjectDetection>(
            "nssc/bottle_coordinates", 10, std::bind(&MessageHandler::bottle_callback, this, std::placeholders::_1));
}

void MessageHandler::bottle_callback(const nssc_interface::msg::ObjectDetection::SharedPtr msg) const
{
    RCLCPP_INFO(this->node->get_logger(), "Message received");
    std::vector<Bottle> bottles;
    for (auto & bottle_m : msg->bottles)
    {
        Bottle new_bottle;
        new_bottle.coord_3d << bottle_m.coord_3d[1], bottle_m.coord_3d[0], bottle_m.coord_3d[2];
        new_bottle.right_coord_2d << bottle_m.right_coord_2d[0], bottle_m.right_coord_2d[1];
        new_bottle.left_coord_2d << bottle_m.left_coord_2d[0], bottle_m.left_coord_2d[1];
        new_bottle.id = bottle_m.id;
        bottles.emplace_back(new_bottle);
    }
    this->moveit_interface->updateScene(bottles);
}

