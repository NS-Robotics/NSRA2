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

#ifndef NSSC_TASK_EXECUTOR_
#define NSSC_TASK_EXECUTOR_

#include "nssc_errors.h"
#include "node.h"
#include "camera_manager.h"
#include "ndi.h"
#include "ingest_api.h"
#include "calibration.h"
#include "triangulation_interface.h"
#include "frame_manager.h"
#include "object_detection.h"

namespace nssc
{
    namespace application
    {
        class Executor : public NSSC_ERRORS
        {
        public:
            Executor(std::shared_ptr<ros::NSSC> &node,
                     std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> &node_executor);
            //executor
            void init();
            void exit();
            void cancel();

            //send_api
            void toggleNDI(bool mono_stream);
            void toggleNDIsource(NSSC_NDI_SEND type);

            //callibration_api
            void runIngest();
            void runCalibration(char *setName);

            //process_api
            void runTriangulation(char *setName);
            void findTriangulationOrigin();
            void runDetection();
            void setColorFilterParams(ColorFilterParams color_filter_params);

            //ingest_api
            void setExposure(float exposure_time);
            void setGain(float gain);


        private:
            std::shared_ptr<ros::NSSC> node;
            std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> node_executor;
            std::shared_ptr<ingest::CameraManager> cam_manager;
            std::shared_ptr<send::NDI> ndi;
            std::unique_ptr<send::FrameManager> frame_manager;

            stereocalibration::Ingest *ingest;
            stereocalibration::Calibration *calibration;
            std::shared_ptr<process::TriangulationInterface> triangulation_interface;
            std::unique_ptr<process::ObjectDetection> object_detection;

            bool ndi_initialized = false;
            bool cam_manager_initialized = false;
            bool ndi_running = false;
            bool triangulation_initialized = false;
            bool detection_running = false;
            bool detection_initalized = false;

            std::string msg_caller = "Executor";

            bool is_closed = false;
        };
    }
}

#endif //NSSC_TASK_EXECUTOR_