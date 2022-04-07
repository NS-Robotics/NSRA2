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

#include "nssc.h"

nssc::NSSC::NSSC(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    this->node_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    auto node = std::make_shared<nssc::ros::NSSC>();
    this->node_executor->add_node(node);

    nssc::NSSC_ERRORS eHandler(node);

    this->executor = std::make_shared<nssc::application::Executor>(node, this->node_executor);
    this->executor->init();

    this->cli = std::make_unique<nssc::application::CLI>(node, this->executor);

    auto message_handler = std::make_shared<nssc::application::MessageHandler>(this->executor);
    this->node_executor->add_node(message_handler);
}

nssc::NSSC::~NSSC()
{
    exit();
}

void nssc::NSSC::spin()
{
    this->node_executor->spin();
}

void nssc::NSSC::exit()
{
    if (this->is_running)
    {
        this->is_running = false;

        this->cli->closeCLI();
        this->executor->exit();
        rclcpp::shutdown();
    }
}