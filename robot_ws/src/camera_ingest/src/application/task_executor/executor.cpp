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

#include "executor.h"
#include "ndi.h"
#include "object_detection.h"
#include "camera_manager.h"
#include "node.h"
#include "frame_manager.h"
#include "ingest_api.h"
#include "calibration.h"
#include "triangulation_interface.h"
#include "nssc_errors.h"

nssc::application::Executor::Executor(std::shared_ptr<ros::NSSC> &node, std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> &node_executor) : NSSC_ERRORS(node)
{
    this->node = node;
    this->node_executor = node_executor;
}

void nssc::application::Executor::exit()
{
    if (this->is_closed) { return; }
    this->is_closed = true;

    if (this->node->g_config.ingest_config.is_running)
    {
        this->ingest->cancelIngest();
    }
    if (this->detection_running)
    {
        this->object_detection->stopDetection();
        this->detection_running = false;
    }
    if (this->detection_initalized)
    {
        this->object_detection->closeDetection();
        this->detection_initalized = false;
    }
    if (this->ndi_running)
    {
        this->ndi->endStream();
        this->ndi_running = false;
    }
    if (this->ndi_initialized)
    {
        this->ndi->closeNDI();
        this->ndi_initialized = false;
    }
    if (this->cam_manager_initialized)
    {
        this->cam_manager->closeCameras();
        this->cam_manager_initialized = false;
    }
    this->node->printInfo(this->msg_caller, "Shutdown complete");
    this->node_executor->cancel();
}

void nssc::application::Executor::init()
{
    this->cam_manager = std::make_shared<ingest::CameraManager>(this->node);
    this->cam_manager->init();
    this->cam_manager->loadCameras();
    this->cam_manager_initialized = true;

    this->frame_manager = send::FrameManager::make_frame(NDI_SEND_RAW);
    this->frame_manager->init(this->node, this->cam_manager);

    this->ndi = std::make_shared<nssc::send::NDI>(this->node, &this->frame_manager);
    this->ndi->init();
    this->ndi_initialized = true;
}

void nssc::application::Executor::cancel()
{
    if (this->node->g_config.ingest_config.is_running)
    {
        this->ingest->cancelIngest();
    }
    if (this->detection_running)
    {
        this->object_detection->stopDetection();
        this->detection_running = false;

        this->ndi->endStream();
        this->ndi_running = false;
    }
}