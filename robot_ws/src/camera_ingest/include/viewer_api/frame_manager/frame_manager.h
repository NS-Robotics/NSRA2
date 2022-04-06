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

#ifndef NSSC_FRAME_MANAGER_H_
#define NSSC_FRAME_MANAGER_H_

#include "camera_manager.h"
#include "nssc_errors.h"
#include "node.h"
#include "blockingconcurrentqueue.h"
#include "stereo_frame.h"

namespace nssc
{
    namespace send
    {
        class FrameManager
        {
        public:
            static std::unique_ptr<FrameManager> make_frame(NSSC_NDI_SEND type);
            virtual void init(std::shared_ptr<ros::NSSC> &node, std::shared_ptr<ingest::CameraManager> &camManager) = 0;

            virtual framestruct::StereoFrame *getFrame() = 0;
            virtual framestruct::StereoFrame *getCameraFrame() = 0;
            virtual void sendFrame(framestruct::StereoFrame* stereoFrame) = 0;
            virtual NSSC_STATUS returnBuf(framestruct::StereoFrame* stereoFrame) = 0;

        protected:
            std::shared_ptr<ros::NSSC>           node;
            std::shared_ptr<ingest::CameraManager>  camManager;

            moodycamel::BlockingConcurrentQueue<framestruct::StereoFrame*> buf_filled;
            std::atomic<int>  num_filled{0};
            std::atomic<bool>  go_get{true};
        };
    }
}

#endif //NSSC_FRAME_MANAGER_H_
