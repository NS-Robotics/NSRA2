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

nssc::application::MessageHandler::MessageHandler(std::shared_ptr<ros::NSSC> &node, std::shared_ptr<Executor> &executor)
{
    this->node = node;
    this->executor = executor;

    this->color_filter_subscriber = this->node->create_subscription<nssc_interface::msg::ColorFilterParams>(
            "color_filter_params", 10, std::bind(&MessageHandler::color_filter_callback, this, std::placeholders::_1));

    this->camera_settings_subscriber = this->node->create_subscription<nssc_interface::msg::CameraSettings>(
            "camera_settings", 10, std::bind(&MessageHandler::camera_settings_callback, this, std::placeholders::_1));
}

void nssc::application::MessageHandler::color_filter_callback(const nssc_interface::msg::ColorFilterParams::SharedPtr msg) const
{
    ColorFilterParams color_filter_params;
    color_filter_params.low_H = msg->low_h;
    color_filter_params.low_S = msg->low_s;
    color_filter_params.low_V = msg->low_v;
    color_filter_params.high_H = msg->high_h;
    color_filter_params.high_S = msg->high_s;
    color_filter_params.high_V = msg->high_v;
    color_filter_params.dilation_element = msg->dilation_element;
    color_filter_params.dilation_size = msg->dilation_size;
    color_filter_params.enable_detection = msg->enable_detection;
    color_filter_params.enable_ndi = msg->enable_ndi;

    this->executor->setColorFilterParams(color_filter_params);
}

void nssc::application::MessageHandler::camera_settings_callback(const nssc_interface::msg::CameraSettings::SharedPtr msg) const
{
    this->executor->setGain(float(msg->gain));
    this->executor->setExposure(float(msg->exposure));
}