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
#include "node.h"
#include "stereo_frame.h"
#include "triangulation_interface.h"

nssc::process::ObjectDetection::ObjectDetection(std::shared_ptr<ros::NSSC> &node,
                                                std::shared_ptr<TriangulationInterface> &triangulation_interface)
{
    this->triangulation_interface = triangulation_interface;
    this->node = node;

    runDetection();
}

nssc::process::ObjectDetection::~ObjectDetection()
{
    if(!this->is_closed) { closeDetection(); }
}

void nssc::process::ObjectDetection::runDetection()
{
    if (!this->detection_running.load())
    {
        this->detection_running = true;
        this->d_thread = std::thread(&ObjectDetection::_detectionThread, this);

        this->node->printInfo(this->msg_caller, "Object Detection started");
    }
    else
    {
        this->node->printWarning(this->msg_caller, "Object Detection already running");
    }
}

void nssc::process::ObjectDetection::closeDetection()
{
    if(this->is_closed) { return; }
    this->is_closed = true;

    if (this->detection_running.load())
        stopDetection();

    this->node->printInfo(this->msg_caller, "Object Detection closed");
}

void nssc::process::ObjectDetection::stopDetection()
{
    this->detection_running = false;
    this->d_thread.join();
    this->node->printInfo(this->msg_caller, "Object Detection stopped");
}

std::string vectorContent(std::vector<float> v){
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

void nssc::process::ObjectDetection::_detectionThread()
{
    framestruct::StereoFrame *stereo_frame;

    cv::Size mono_size = cv::Size(this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res);

    cv::Mat left_rgb, left_hsv, left_thresh, right_rgb, right_hsv, right_thresh;

    int low_H = 105, low_S = 120, low_V = 100;
    int high_H = 120, high_S = 220, high_V = 255;

    cv::SimpleBlobDetector detector;
    std::vector<cv::KeyPoint> keypoints;

    while(this->detection_running.load())
    {
        stereo_frame = this->triangulation_interface->getFrame();

        cv::Mat left_inp(mono_size, CV_8UC4, stereo_frame->left_camera->frame_buf.hImageBuf);
        cv::Mat right_inp(mono_size, CV_8UC4, stereo_frame->right_camera->frame_buf.hImageBuf);

        cv::cvtColor(left_inp, left_rgb, cv::COLOR_RGBA2RGB);
        cv::cvtColor(right_inp, right_rgb, cv::COLOR_RGBA2RGB);

        cv::cvtColor(left_rgb, left_hsv, cv::COLOR_RGB2HSV);
        cv::cvtColor(right_rgb, right_hsv, cv::COLOR_RGB2HSV);

        cv::inRange(left_hsv,  cv::Scalar(high_H, high_S, high_V), cv::Scalar(low_H, low_S, low_V), left_thresh);
        cv::inRange(right_hsv, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), right_thresh);

        /*
        cv::Mat lower_red_hue_range;
        cv::Mat upper_red_hue_range;
        cv::inRange(left_hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
        cv::inRange(left_hsv, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);

        cv::Mat red_hue_image;
        cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);

        cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);
        */
        cv::cvtColor(left_thresh, left_inp, cv::COLOR_GRAY2RGBA);

        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(left_thresh, circles, cv::HOUGH_GRADIENT, 1, left_thresh.rows / 8, 100, 20, 0, 0);

        if (circles.size() > 0)
        {
            for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
                cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
                int radius = std::round(circles[current_circle][2]);

                cv::circle(left_inp, center, radius, cv::Scalar(0, 255, 0), 2);
            }
        }

        /*
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
                    vectorContent(print_v),
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
         */

        this->triangulation_interface->sendFrame(stereo_frame);
    }
}

void nssc::process::ObjectDetection::_testDetectionThread()
{
    framestruct::StereoFrame *stereo_frame;

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

    cv::Size mono_size = cv::Size(this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res);

    cv::Mat left_conv;
    cv::Mat right_conv;

    while(this->detection_running.load())
    {
        stereo_frame = this->triangulation_interface->getFrame();

        cv::Mat left_inp(mono_size, CV_8UC4, stereo_frame->left_camera->frame_buf.hImageBuf);
        cv::Mat right_inp(mono_size, CV_8UC4, stereo_frame->right_camera->frame_buf.hImageBuf);

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
        //std::cout << vectorContent(print_v) << std::endl;

        cv::circle(left_inp, left_originMarkers[0], 10, (0,0,255), 2);
        cv::circle(right_inp, right_originMarkers[0], 10, (0,0,255), 2);
        cv::putText(left_inp,
                    vectorContent(print_v),
                    cv::Point2f(left_originMarkers[0].x - 100, left_originMarkers[0].y - 30),
                    cv::FONT_HERSHEY_COMPLEX_SMALL,
                    1.4,
                    cv::Scalar(255, 0, 0),
                    2,
                    cv::LINE_AA);
        cv::putText(right_inp,
                    vectorContent(print_v),
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