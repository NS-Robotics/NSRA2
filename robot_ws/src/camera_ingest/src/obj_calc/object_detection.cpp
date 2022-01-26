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

#include <iostream>
#include "object_detection.h"

ObjectDetection::ObjectDetection(std::shared_ptr<NSSC> &node,
                                 std::shared_ptr<TriangulationInterface> &triangulation_interface)
{
    this->triangulation_interface = triangulation_interface;
    this->node = node;

    this->detection_running = true;
    this->d_thread = std::thread(&ObjectDetection::detectionThread, this);

    this->node->printInfo(this->msg_caller, "Object Detection started");
}

ObjectDetection::~ObjectDetection()
{
    if(!this->is_closed) { closeDetection(); }
}

void ObjectDetection::closeDetection()
{
    if(this->is_closed) { return; }
    this->is_closed = true;

    if (this->detection_running.load())
        stopDetection();

    this->node->printInfo(this->msg_caller, "Object Detection closed");
}

void ObjectDetection::stopDetection()
{
    this->detection_running = false;
    this->d_thread.join();
    this->node->printInfo(this->msg_caller, "Object Detection stopped");
}

std::string vector_content(std::vector<float> v){
    std::string s;
    s += '[';
    for (int i = 0; i < v.size(); i++){
        s += std::to_string((int) std::round(v[i])) ;
        if (v[i] != v.back())
            s += ", ";
    }
    s += ']';
    return s;
}

void ObjectDetection::detectionThread()
{
    stereoFrame *stereo_frame;

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

    cv::Size mono_size = cv::Size(this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res);

    cv::Mat left_conv;
    cv::Mat right_conv;

    while(this->detection_running.load())
    {
        stereo_frame = this->triangulation_interface->getFrame();

        cv::Mat left_inp(mono_size, CV_8UC4, stereo_frame->leftCamera->frameBuf.hImageBuf);
        cv::Mat right_inp(mono_size, CV_8UC4, stereo_frame->rightCamera->frameBuf.hImageBuf);

        cv::cvtColor(left_inp, left_conv, cv::COLOR_RGBA2GRAY);
        cv::cvtColor(right_inp, right_conv, cv::COLOR_RGBA2GRAY);

        std::vector<int> left_markerIds, right_markerIds;
        std::vector<std::vector<cv::Point2f>> left_markerCorners, right_markerCorners, left_rejectedCandidates, right_rejectedCandidates;

        cv::aruco::detectMarkers(left_conv, dictionary, left_markerCorners,
                                 left_markerIds,parameters, left_rejectedCandidates);
        cv::aruco::detectMarkers(right_conv, dictionary, right_markerCorners,
                                 right_markerIds,parameters, right_rejectedCandidates);

        if (left_markerIds.empty() || right_markerIds.empty())
        {
            this->triangulation_interface->sendFrame(stereo_frame);
            continue;
        }

        int left_idx, right_idx;
        auto left_element = std::find(left_markerIds.begin(), left_markerIds.end(), 6);
        auto right_element = std::find(right_markerIds.begin(), right_markerIds.end(), 6);

        if (left_element == left_markerIds.end() || right_element == right_markerIds.end())
        {
            this->triangulation_interface->sendFrame(stereo_frame);
            continue;
        }

        left_idx = left_element - left_markerIds.begin();
        right_idx = right_element - right_markerIds.begin();

        std::vector<cv::Point2f> left_originMarkers, right_originMarkers;
        left_originMarkers.push_back(left_markerCorners[left_idx][0]);
        right_originMarkers.push_back(right_markerCorners[right_idx][0]);

        std::vector<Eigen::Vector3d> coords_3d = this->triangulation_interface->triangulatePoints(left_originMarkers, right_originMarkers);

        std::vector<float> print_v = {static_cast<float>(coords_3d[0][0]), static_cast<float>(coords_3d[0][1]), static_cast<float>(coords_3d[0][2])};
        //std::cout << vector_content(print_v) << std::endl;

        cv::circle(left_inp, left_originMarkers[0], 10, (0,0,255), 2);
        cv::circle(right_inp, right_originMarkers[0], 10, (0,0,255), 2);
        cv::putText(left_inp,
                    vector_content(print_v),
                    cv::Point2f(left_originMarkers[0].x - 100, left_originMarkers[0].y - 30),
                    cv::FONT_HERSHEY_COMPLEX_SMALL,
                    1.4,
                    cv::Scalar(255, 0, 0),
                    2,
                    cv::LINE_AA);
        cv::putText(right_inp,
                    vector_content(print_v),
                    cv::Point2f(right_originMarkers[0].x - 100, right_originMarkers[0].y - 30),
                    cv::FONT_HERSHEY_COMPLEX_SMALL,
                    1.4,
                    cv::Scalar(255, 0, 0),
                    2,
                    cv::LINE_AA);

        std::vector<cv::Point2f> left_origin, right_origin;
        std::tie(left_origin, right_origin) = this->triangulation_interface->getOrigin();

        for(int i = 0; i < left_origin.size(); i++)
        {
            cv::circle(left_inp, left_origin[i], 10, cv::Scalar(150, 255, 0), 2);
            cv::circle(right_inp, right_origin[i], 10, cv::Scalar(150, 255, 0), 2);
        }

        this->triangulation_interface->sendFrame(stereo_frame);
    }
}