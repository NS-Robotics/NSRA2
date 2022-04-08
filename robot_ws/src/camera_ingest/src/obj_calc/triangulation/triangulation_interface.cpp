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

#include "triangulation_interface.h"
#include "node.h"
#include "stereo_frame.h"
#include "frame_manager.h"
#include "triangulation.h"

nssc::process::TriangulationInterface::TriangulationInterface(std::shared_ptr<nssc::ros::NSSC> &node, std::unique_ptr<nssc::send::FrameManager> *frameManager,
                                                              const char *setName) : Triangulation(node, frameManager, setName)
{
    this->rectified_frame = nssc::framestruct::StereoFrame::makeFrame(this->node->g_config.frameConfig.g_type);
    this->rectified_frame->alloc(this->node);
}

nssc::framestruct::StereoFrame *nssc::process::TriangulationInterface::getFrame()
{
    nssc::framestruct::StereoFrame *stereo_frame;

    stereo_frame = (*this->frame_manager)->getCameraFrame();

    return stereo_frame;
}

std::vector<Eigen::Vector3d> nssc::process::TriangulationInterface::triangulatePoints(std::vector<cv::Point2f> &left_2D, std::vector<cv::Point2f> &right_2D)
{
    std::vector<cv::Point2f> left_2D_undistort, right_2D_undistort;
    cv::undistortPoints(left_2D, left_2D_undistort, this->left_K, this->left_D, this->left_R, this->left_P);
    cv::undistortPoints(right_2D, right_2D_undistort, this->right_K, this->right_D, this->right_R, this->right_P);

    cv::Mat p_4d_origin(4,left_2D.size(),CV_64F);
    cv::triangulatePoints(this->left_P, this->right_P, left_2D_undistort, right_2D_undistort, p_4d_origin);

    std::vector<Eigen::Vector3d> p_3d_origin;
    for (int i = 0; i < left_2D.size(); i++)
    {
        p_3d_origin.emplace_back( p_4d_origin.at<float>(0,i) / p_4d_origin.at<float>(3,i),
                                  p_4d_origin.at<float>(1,i) / p_4d_origin.at<float>(3,i),
                                  p_4d_origin.at<float>(2,i) / p_4d_origin.at<float>(3,i) );
    }

    return _transform_coordinates(p_3d_origin);
}

Eigen::Vector3d nssc::process::TriangulationInterface::triangulatePoints(cv::Point2f &left_2D, cv::Point2f &right_2D)
{
    std::vector<cv::Point2f> left_2D_vec = {left_2D};
    std::vector<cv::Point2f> right_2D_vec = {right_2D};
    std::vector<cv::Point2f> left_2D_undistort, right_2D_undistort;
    cv::undistortPoints(left_2D_vec, left_2D_undistort, this->left_K, this->left_D, this->left_R, this->left_P);
    cv::undistortPoints(right_2D_vec, right_2D_undistort, this->right_K, this->right_D, this->right_R, this->right_P);

    cv::Mat p_4d_origin(4,1,CV_64F);
    cv::triangulatePoints(this->left_P, this->right_P, left_2D_undistort, right_2D_undistort, p_4d_origin);

    Eigen::Vector3d p_3d_origin( p_4d_origin.at<float>(0,0) / p_4d_origin.at<float>(3,0),
                                 p_4d_origin.at<float>(1,0) / p_4d_origin.at<float>(3,0),
                                 p_4d_origin.at<float>(2,0) / p_4d_origin.at<float>(3,0) );

    return _transform_coordinates(p_3d_origin);
}

void nssc::process::TriangulationInterface::sendFrame(nssc::framestruct::StereoFrame *stereo_frame)
{
    (*this->frame_manager)->sendFrame(stereo_frame);
}

void nssc::process::TriangulationInterface::returnBuf(nssc::framestruct::StereoFrame *stereo_frame)
{
    (*this->frame_manager)->returnBuf(stereo_frame);
}

std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>> nssc::process::TriangulationInterface::getOrigin()
{
    return std::make_tuple(this->left_origin_pts, this->right_origin_pts);
}
