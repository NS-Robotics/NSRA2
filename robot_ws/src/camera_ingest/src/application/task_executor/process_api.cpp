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

void nssc::application::Executor::runTriangulation(char *setName)
{
    toggleNDIsource(NDI_SEND_TRIANGULATION);

    this->triangulation_interface = std::make_shared<process::TriangulationInterface>(this->node, &this->frame_manager, setName);
    if (this->triangulation_interface->init() == NSSC_STATUS_SUCCESS)
        this->triangulation_initialized = true;
    else
        this->triangulation_initialized = false;

    this->ndi->endStream();
    this->ndi_running = false;
}

void nssc::application::Executor::runDetection()
{
    if (this->detection_initalized && !this->detection_running)
    {
        toggleNDIsource(NDI_SEND_TRIANGULATION);
        this->object_detection->runDetection();
        this->detection_running = true;
    }
    else if (this->triangulation_initialized && !this->detection_initalized)
    {
        toggleNDIsource(NDI_SEND_TRIANGULATION);
        this->object_detection = std::make_unique<process::ObjectDetection>(this->node, this->triangulation_interface);
        this->detection_running = true;
        this->detection_initalized = true;
    }
    else if (!this->triangulation_initialized && !this->detection_initalized)
    {
        toggleNDIsource(NDI_SEND_TRIANGULATION);

        this->triangulation_interface = std::make_shared<process::TriangulationInterface>(this->node, &this->frame_manager, this->node->g_config.triangulationConfig.standard_config_file);
        if (this->triangulation_interface->init() != NSSC_STATUS_SUCCESS)
        {
            this->triangulation_initialized = false;
            return;
        }
        this->triangulation_initialized = true;

        this->object_detection = std::make_unique<process::ObjectDetection>(this->node, this->triangulation_interface);
        this->detection_running = true;
        this->detection_initalized = true;
    }
}

void nssc::application::Executor::findTriangulationOrigin()
{
    if (this->detection_running)
    {
        this->object_detection->stopDetection();

        if (this->triangulation_interface->findOrigin() == NSSC_STATUS_SUCCESS)
            this->object_detection->runDetection();
        else
        {
            this->detection_running = false;

            this->ndi->endStream();
            this->ndi_running = false;
        }
    }
    else if (this->triangulation_initialized)
    {
        toggleNDIsource(NDI_SEND_TRIANGULATION);

        this->triangulation_interface->findOrigin();

        this->ndi->endStream();
        this->ndi_running = false;
    }
    else
        this->node->printError(this->msg_caller, "Triangulation Interface not initialized!");
}

void nssc::application::Executor::setColorFilterParams(ColorFilterParams color_filter_params)
{
    this->node->g_config.triangulationConfig.color_filter_params = color_filter_params;
    if (this->detection_running)
    {
        this->object_detection->setColorFilterParams();
    }
}