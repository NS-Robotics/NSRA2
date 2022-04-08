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

#ifndef CAMERA_INGEST_MSG_PUBLISHER_H_
#define CAMERA_INGEST_MSG_PUBLISHER_H_

#include "node.h"
#include "triangulation_interface.h"

#include "camera_ingest/msg/object_detection.hpp"
#include "camera_ingest/msg/bottle.hpp"

namespace nssc
{
    namespace process
    {
        struct Bottle
        {
            Eigen::Vector3d coord_3d;
            cv::Point2f left_coord_2d;
            cv::Point2f right_coord_2d;
            int id;
        };

        class DetectionPublisher
        {
        public:
            DetectionPublisher(std::shared_ptr<ros::NSSC> &node);
            void publishBottleCoordinates(std::vector<Bottle> bottles);

        private:
            std::shared_ptr<ros::NSSC> node;

            rclcpp::Publisher<camera_ingest::msg::ObjectDetecion>::SharedPtr bottle_publisher;
        };
    }
}

#endif //CAMERA_INGEST_MSG_PUBLISHER_H_
