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

#include "frame_manager.h"
#include "camera_manager.h"
#include "node.h"
#include "stereo_frame.h"
#include "nssc_errors.h"

class sendRaw: public nssc::send::FrameManager
{
public:
    void init(std::shared_ptr<nssc::ros::NSSC> &node, std::shared_ptr<nssc::ingest::CameraManager> &camManager) override
    {
        this->node = node;
        this->camManager = camManager;
    }

    nssc::framestruct::StereoFrame *getCameraFrame() override
    {
    }

    nssc::framestruct::StereoFrame *getFrame() override
    {
        return this->camManager->getFrame();;
    }

    void sendFrame(nssc::framestruct::StereoFrame* stereoFrame) override
    {
    }

    nssc::NSSC_STATUS returnBuf(nssc::framestruct::StereoFrame* stereo_frame) override
    {
        return this->camManager->returnBuf(stereo_frame);
    }
};

class sendTriangulation : public nssc::send::FrameManager
{
public:
    void init(std::shared_ptr<nssc::ros::NSSC> &node, std::shared_ptr<nssc::ingest::CameraManager> &camManager) override
    {
        this->node = node;
        this->camManager = camManager;
    }

    nssc::framestruct::StereoFrame *getCameraFrame() override
    {
        while(!this->go_get.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        this->go_get = false;
        return this->camManager->getFrame();
    }

    nssc::framestruct::StereoFrame *getFrame() override
    {
        nssc::framestruct::StereoFrame* stereo_frame;
        while(this->node->g_config.frame_config.stream_on)
        {
            if (this->buf_filled.wait_dequeue_timed(stereo_frame, std::chrono::seconds(1)))
            {
                this->num_filled--;
                return stereo_frame;
            }
        }
        return nullptr;
    }

    void sendFrame(nssc::framestruct::StereoFrame* stereo_frame) override
    {
        if (this->node->g_config.frame_config.stream_on)
        {
            stereo_frame->process(this->node->g_config.frame_config.resize_frame);
            this->buf_filled.enqueue(stereo_frame);
            this->num_filled++;
        }
        else
        {
            this->go_get = true;
            this->camManager->returnBuf(stereo_frame);
        }
    }

    nssc::NSSC_STATUS returnBuf(nssc::framestruct::StereoFrame* stereo_frame) override
    {
        this->go_get = true;
        return this->camManager->returnBuf(stereo_frame);
    }
};

class sendIngest : public nssc::send::FrameManager
{
public:
    void init(std::shared_ptr<nssc::ros::NSSC> &node, std::shared_ptr<nssc::ingest::CameraManager> &camManager) override
    {
        this->node = node;
        this->camManager = camManager;
    }

    nssc::framestruct::StereoFrame *getCameraFrame() override
    {
        while(!this->go_get.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        this->go_get = false;
        return this->camManager->getFrame();
    }

    nssc::framestruct::StereoFrame *getFrame() override
    {
        nssc::framestruct::StereoFrame* stereo_frame;
        while(this->node->g_config.frame_config.stream_on)
        {
            if (this->buf_filled.wait_dequeue_timed(stereo_frame, std::chrono::seconds(1)))
            {
                this->num_filled--;
                return stereo_frame;
            }
        }
        return nullptr;
    }

    void sendFrame(nssc::framestruct::StereoFrame* stereo_frame) override
    {
        this->buf_filled.enqueue(stereo_frame);
        this->num_filled++;
    }

    nssc::NSSC_STATUS returnBuf(nssc::framestruct::StereoFrame* stereoFrame) override
    {
        this->go_get = true;
        return this->camManager->returnBuf(stereoFrame);
    }
};

//TODO: implementation
class sendCalibration : public nssc::send::FrameManager
{
public:
    void init(std::shared_ptr<nssc::ros::NSSC> &node, std::shared_ptr<nssc::ingest::CameraManager> &camManager) override
    {
        this->node = node;
        this->camManager = camManager;
    }

    nssc::framestruct::StereoFrame *getCameraFrame() override
    {
    }

    nssc::framestruct::StereoFrame *getFrame() override
    {
    }

    void sendFrame(nssc::framestruct::StereoFrame* stereoFrame) override
    {

    }

    nssc::NSSC_STATUS returnBuf(nssc::framestruct::StereoFrame* stereoFrame) override
    {
        return this->camManager->returnBuf(stereoFrame);
    }
};

std::unique_ptr<nssc::send::FrameManager> nssc::send::FrameManager::make_frame(NSSC_NDI_SEND type)
{
    switch(type)
    {
        case NDI_SEND_RAW:
            return std::make_unique<sendRaw>();
        case NDI_SEND_TRIANGULATION:
            return std::make_unique<sendTriangulation>();
        case NDI_SEND_INGEST:
            return std::make_unique<sendIngest>();
        case NDI_SEND_CALIBRATION:
            return std::make_unique<sendCalibration>();
    }
}