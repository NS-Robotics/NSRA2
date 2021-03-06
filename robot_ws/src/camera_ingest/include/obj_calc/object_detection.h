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

#ifndef NSSC_OBJECT_DETECTION_H_
#define NSSC_OBJECT_DETECTION_H_

#include "node.h"
#include "triangulation_interface.h"
#include "msg_publisher.h"
#include "frame_struct.h"

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaarithm.hpp>
#include "opencv2/cudafilters.hpp"

namespace nssc
{
    namespace process
    {
        class ObjectDetection
        {
        public:
            ObjectDetection(std::shared_ptr<ros::NSSC> &node,
                            std::shared_ptr<TriangulationInterface> &triangulation_interface);
            ~ObjectDetection();
            void stopDetection();
            void closeDetection();
            void runDetection();
            void setColorFilterParams();

        private:
            std::shared_ptr<TriangulationInterface> triangulation_interface;
            std::shared_ptr<ros::NSSC> node;
            std::unique_ptr<DetectionPublisher> detection_publisher;

            std::string msg_caller = "Detection";

            ColorFilterParams color_filter_params;
            cv::Mat kernel;

            void _detectionThread();
            void _testDetectionThread();

            cv::cuda::GpuMat _cudaInRange(const cv::cuda::GpuMat& src);
            std::vector<Bottle> _processKeypoints(std::vector<cv::KeyPoint> keypoints_left, std::vector<cv::KeyPoint> keypoints_right);
            static std::vector<std::pair<cv::KeyPoint, cv::KeyPoint>> _getClosestPairs(std::vector<cv::KeyPoint> v1, std::vector<cv::KeyPoint> v2);

            std::atomic<bool> detection_running{false};
            std::thread d_thread;

            bool is_closed = false;
        };
    }
}

#endif //NSSC_OBJECT_DETECTION_H_