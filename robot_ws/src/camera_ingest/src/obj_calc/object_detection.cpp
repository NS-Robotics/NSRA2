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

void nssc::process::ObjectDetection::setColorFilterParams()
{
    this->color_filter_params = this->node->g_config.triangulationConfig.color_filter_params;

    this->kernel = cv::getStructuringElement(this->color_filter_params.dilation_element,
                                             cv::Size(this->color_filter_params.dilation_size + 1,
                                                      this->color_filter_params.dilation_size + 1),
                                             cv::Point(this->color_filter_params.dilation_size,
                                                       this->color_filter_params.dilation_size));
}

std::string type2str(int type) {
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }

    r += "C";
    r += (std::to_string(chans+'0'));

    return r;
}

void nssc::process::ObjectDetection::_detectionThread()
{
    setColorFilterParams();

    framestruct::StereoFrame *stereo_frame;

    cv::Size mono_size = cv::Size(this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res);

    cv::Mat left_rgb, left_hsv, left_thresh, left_bitw, left_dilate, left_keyp,
            right_rgb, right_hsv, right_thresh, right_bitw, right_dilate, right_keyp;

    cv::SimpleBlobDetector::Params params;

    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 1000;
    params.maxArea = 100000;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.1;

    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0;
    params.maxConvexity = 1;

    // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = 0.1;

    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    std::vector<cv::KeyPoint> keypoints_left;
    std::vector<cv::KeyPoint> keypoints_right;

    std::vector<Eigen::Vector3d> coords_3d;
    std::vector<cv::Point2f> left_coords, right_coords;

    cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6,6));

    while(this->detection_running.load())
    {
        stereo_frame = this->triangulation_interface->getFrame();

        cv::Mat left_inp(mono_size, CV_8UC4, stereo_frame->left_camera->frame_buf.hImageBuf);
        cv::Mat right_inp(mono_size, CV_8UC4, stereo_frame->right_camera->frame_buf.hImageBuf);

        cv::cvtColor(left_inp, left_rgb, cv::COLOR_RGBA2RGB);
        cv::cvtColor(right_inp, right_rgb, cv::COLOR_RGBA2RGB);

        cv::cvtColor(left_rgb, left_hsv, cv::COLOR_RGB2HSV);
        cv::cvtColor(right_rgb, right_hsv, cv::COLOR_RGB2HSV);

        cv::inRange(left_hsv,
                    cv::Scalar(this->color_filter_params.low_H, this->color_filter_params.low_S, this->color_filter_params.low_V),
                    cv::Scalar(this->color_filter_params.high_H, this->color_filter_params.high_S, this->color_filter_params.high_V),
                    left_hsv);
        cv::inRange(right_hsv,
                    cv::Scalar(this->color_filter_params.low_H, this->color_filter_params.low_S, this->color_filter_params.low_V),
                    cv::Scalar(this->color_filter_params.high_H, this->color_filter_params.high_S, this->color_filter_params.high_V),
                    right_hsv);

        cv::erode(left_hsv, left_hsv, kernel);
        cv::erode(right_hsv, right_hsv, kernel);

        cv::bitwise_not(left_hsv, left_hsv);
        cv::bitwise_not(right_hsv, right_hsv);

        cv::morphologyEx(left_hsv, left_hsv, cv::MORPH_OPEN, morph_kernel);
        cv::morphologyEx(right_hsv, right_hsv, cv::MORPH_OPEN, morph_kernel);

        if (this->color_filter_params.enable_detection)
        {
            detector->detect(left_hsv, keypoints_left);
            detector->detect(right_hsv, keypoints_right);

            if (!keypoints_left.empty() && !keypoints_right.empty())
            {
                for (auto & i : keypoints_left)
                {
                    left_coords.push_back(i.pt);
                }

                for (auto & i : keypoints_right)
                {
                    right_coords.push_back(i.pt);
                }

                coords_3d = this->triangulation_interface->triangulatePoints(left_coords, right_coords);

                /*
                for (auto & i : coords_3d)
                {
                    std::cout << i << std::endl;
                }
                 */
                if (this->color_filter_params.enable_ndi)
                {
                    std::vector<float> text_right = {static_cast<float>(coords_3d[0][0]), static_cast<float>(coords_3d[0][1]), static_cast<float>(coords_3d[0][2])};
                    std::vector<float> text_left = {static_cast<float>(coords_3d[0][0]), static_cast<float>(coords_3d[0][1]), static_cast<float>(coords_3d[0][2])};

                    cv::putText(left_inp,
                                vectorContent(text_right),
                                cv::Point2f(left_coords[0].x - 100, left_coords[0].y - 70),
                                cv::FONT_HERSHEY_COMPLEX_SMALL,
                                1.4,
                                cv::Scalar(255, 0, 0),
                                2,
                                cv::LINE_AA);
                    cv::putText(right_inp,
                                vectorContent(text_left),
                                cv::Point2f(right_coords[0].x - 100, right_coords[0].y - 70),
                                cv::FONT_HERSHEY_COMPLEX_SMALL,
                                1.4,
                                cv::Scalar(255, 0, 0),
                                2,
                                cv::LINE_AA);
                }
            }

            if (this->color_filter_params.enable_ndi)
            {
                cv::drawKeypoints(left_inp, keypoints_left, left_inp, cv::Scalar(255,0,0),
                                  cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

                cv::drawKeypoints(right_inp, keypoints_right, right_inp, cv::Scalar(255,0,0),
                                  cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
            }
        }
        else if (this->color_filter_params.enable_ndi)
        {
            cv::cvtColor(left_hsv, left_inp, cv::COLOR_GRAY2RGBA);
            cv::cvtColor(right_hsv, right_inp, cv::COLOR_GRAY2RGBA);
        }

        if (this->color_filter_params.enable_ndi)
        {
            this->triangulation_interface->sendFrame(stereo_frame);
        }
        else
        {
            this->triangulation_interface->returnBuf(stereo_frame);
        }
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