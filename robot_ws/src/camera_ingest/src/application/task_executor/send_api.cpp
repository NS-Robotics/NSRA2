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

void nssc::application::Executor::toggleNDI(bool mono_stream)
{
    if (this->ndi_running)
    {
        this->ndi->endStream();
        this->ndi_running = false;
    }
    else if (mono_stream == this->node->g_config.frame_config.mono_stream)
    {
        toggleNDIsource(NDI_SEND_RAW);
    }
    else
    {
        if (this->ndi_initialized)
        {
            this->ndi->closeNDI();
            this->ndi_initialized = false;
        }
        this->node->g_config.frame_config.mono_stream = mono_stream;
        this->node->g_config.frame_config.calculate_params();

        this->ndi->init();
        this->ndi_initialized = true;
        this->ndi->startStream();
        this->ndi_running = true;
    }
}

void nssc::application::Executor::toggleNDIsource(NSSC_NDI_SEND type)
{
    if (this->ndi_running)
    {
        this->ndi->endStream();
        this->ndi_running = false;
    }

    this->frame_manager = send::FrameManager::make_frame(type);
    this->frame_manager->init(this->node, this->cam_manager);

    this->ndi->startStream();
    this->ndi_running = true;
}