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

#include "msg_publisher.h"

nssc::process::DetectionPublisher::DetectionPublisher(std::shared_ptr<ros::NSSC> &node)
{
    this->node = node;
    this->bottle_publisher = this->node->create_publisher<camera_ingest::msg::ObjectDetection>("nssc/bottle_coordinates", 10);
}

void nssc::process::DetectionPublisher::publishBottleCoordinates(std::vector<Bottle> bottles)
{
    std::vector<camera_ingest::msg::Bottle> bottles_msg;
    for (auto & bottle : bottles)
    {
        auto bottle_msg = camera_ingest::msg::Bottle();
        bottle_msg.coord_3d = std::array<float, 3>{static_cast<float>(bottle.coord_3d[0]),
                                                   static_cast<float>(bottle.coord_3d[1]),
                                                   static_cast<float>(bottle.coord_3d[2])};
        bottle_msg.left_coord_2d = std::array<float, 2>{bottle.left_coord_2d.x,
                                                        bottle.left_coord_2d.y};
        bottle_msg.right_coord_2d = std::array<float, 2>{bottle.right_coord_2d.x,
                                                         bottle.right_coord_2d.y};
        bottle_msg.id = bottle.id;

        bottles_msg.push_back(bottle_msg);
    }

    auto msg = camera_ingest::msg::ObjectDetection();
    msg.bottles = bottles_msg;
    this->bottle_publisher->publish(msg);
}
