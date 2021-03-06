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

#ifndef CAMERA_INGEST_MESSAGE_HANDLER_H_
#define CAMERA_INGEST_MESSAGE_HANDLER_H_

#include "executor.h"
#include "node.h"

#include "nssc_interface/msg/color_filter_params.hpp"
#include "nssc_interface/msg/camera_settings.hpp"

namespace nssc
{
    namespace application
    {
        class MessageHandler
        {
        public:
            MessageHandler(std::shared_ptr<ros::NSSC> &node, std::shared_ptr<Executor> &executor);

        private:
            void color_filter_callback(const nssc_interface::msg::ColorFilterParams::SharedPtr msg) const;
            void camera_settings_callback(const nssc_interface::msg::CameraSettings::SharedPtr msg) const;

            std::shared_ptr<ros::NSSC> node;
            std::shared_ptr<Executor> executor;

            rclcpp::Subscription<nssc_interface::msg::ColorFilterParams>::SharedPtr color_filter_subscriber;
            rclcpp::Subscription<nssc_interface::msg::CameraSettings>::SharedPtr camera_settings_subscriber;

            std::string msg_caller = "MessageHandler";
        };
    }
}

#endif //CAMERA_INGEST_MESSAGE_HANDLER_H_
