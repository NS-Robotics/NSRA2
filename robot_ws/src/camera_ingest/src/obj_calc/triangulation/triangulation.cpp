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
#include "triangulation.h"
#include "node.h"
#include "stereo_frame.h"
#include "frame_manager.h"
#include "nssc_errors.h"

nssc::process::Triangulation::Triangulation(std::shared_ptr<nssc::ros::NSSC> &node, std::unique_ptr<nssc::send::FrameManager>* frame_manager,
                                            const char *setName)
{
    this->node = node;
    this->frame_manager = frame_manager;
    this->set_path = this->node->g_config.share_dir + "/" + setName + "/";
}

nssc::NSSC_STATUS nssc::process::Triangulation::init()
{
    std::ifstream xmlFile(this->set_path + "config.xml");
    if (xmlFile.fail())
    {
        this->node->printWarning(this->msg_caller, "This configuration doesn't exist!");
        return NSSC_TRIANGULATION_FILE_NOT_FOUND;
    }
    else
    {
        cv::FileStorage config_file(this->set_path + "config.xml", cv::FileStorage::READ);

        config_file["left_K"] >> this->left_K;
        config_file["left_D"] >> this->left_D;
        config_file["right_K"] >> this->right_K;
        config_file["right_D"] >> this->right_D;

        config_file["right_R"] >> this->right_R;
        config_file["left_R"] >> this->left_R;
        config_file["right_P"] >> this->right_P;
        config_file["left_P"] >> this->left_P;

        this->dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        this->parameters = cv::aruco::DetectorParameters::create();

        this->mono_size = cv::Size(this->node->g_config.frame_config.mono_x_res, this->node->g_config.frame_config.mono_y_res);

        cv::initUndistortRectifyMap(this->left_K, this->left_D, this->left_R, this->left_P,
                                    mono_size, CV_32FC1, this->left_map1, this->left_map2);
        cv::initUndistortRectifyMap(this->right_K, this->right_D, this->right_R, this->right_P,
                                    mono_size, CV_32FC1, this->right_map1, this->right_map2);

        NSSC_STATUS status;

        status = findOrigin();

        if (status == NSSC_STATUS_SUCCESS)
            this->node->printInfo(this->msg_caller, "Origin successfully calculated");

        return status;
    }
}

std::string vectorContent(std::vector<int> v){
    std::string s;
    s += '[';
    for (int i = 0; i < v.size(); i++){
        s += "\"";
        s += std::to_string(v[i]) ;
        s += "\"";
        if (v[i] != v.back())
            s += " ,";
    }
    s += ']';
    return s;
}

nssc::NSSC_STATUS nssc::process::Triangulation::findOrigin()
{
    nssc::framestruct::StereoFrame* stereoFrame;

    while (this->node->g_config.frame_config.stream_on)
    {
        stereoFrame = (*this->frame_manager)->getCameraFrame();
        if (stereoFrame->timedif < this->node->g_config.triangulation_config.max_origin_frame_time_diff)
            break;
        else
            (*this->frame_manager)->returnBuf(stereoFrame);
    }

    cv::Mat left_inp(mono_size,CV_8UC3, stereoFrame->left_camera->rgb_buf.hImageBuf);
    cv::Mat right_inp(mono_size, CV_8UC3, stereoFrame->right_camera->rgb_buf.hImageBuf);

    cv::Mat left_conv;
    cv::Mat right_conv;

    cv::cvtColor(left_inp, left_conv, cv::COLOR_RGB2GRAY);
    cv::cvtColor(right_inp, right_conv, cv::COLOR_RGB2GRAY);

    std::vector<int> left_markerIds, right_markerIds;
    std::vector<std::vector<cv::Point2f>> left_markerCorners, right_markerCorners, left_rejectedCandidates, right_rejectedCandidates;

    cv::aruco::detectMarkers(left_conv, this->dictionary, left_markerCorners,
                             left_markerIds,this->parameters, left_rejectedCandidates,
                             this->left_K, this->left_D);
    cv::aruco::detectMarkers(right_conv, this->dictionary, right_markerCorners,
                             right_markerIds,this->parameters, right_rejectedCandidates,
                             this->left_K, this->left_D);

    if (!__originMarkersExist(left_markerIds) || !__originMarkersExist(right_markerIds))
    {
        this->node->printError(this->msg_caller, "Origin markers not found!");
        return NSSC_TRIANGULATION_MARKERS_NOT_FOUND;
    }

    std::vector<cv::Point2f> left_originMarkers, right_originMarkers;
    for(int i = 0; i < 3; i++)
    {
        int left_idx, right_idx;
        auto left_element = std::find(left_markerIds.begin(), left_markerIds.end(), this->node->g_config.triangulation_config.origin_ids[i]);
        left_idx = left_element - left_markerIds.begin();
        auto right_element = std::find(right_markerIds.begin(), right_markerIds.end(), this->node->g_config.triangulation_config.origin_ids[i]);
        right_idx = right_element - right_markerIds.begin();
        left_originMarkers.push_back(left_markerCorners[left_idx][0]);
        right_originMarkers.push_back(right_markerCorners[right_idx][0]);
    }

    this->left_origin_pts = left_originMarkers;
    this->right_origin_pts = right_originMarkers;

    std::vector<cv::Point2f> left_2D_undistort, right_2D_undistort;
    cv::undistortPoints(left_originMarkers, left_2D_undistort, this->left_K, this->left_D, this->left_R, this->left_P);
    cv::undistortPoints(right_originMarkers, right_2D_undistort, this->right_K, this->right_D, this->right_R, this->right_P);

    cv::Mat p_4d_origin(4,3,CV_64F);
    cv::triangulatePoints(this->left_P, this->right_P, left_2D_undistort, right_2D_undistort, p_4d_origin);

    std::vector<Eigen::Vector3d> p_3d_origin;
    for (int i = 0; i < 3; i++)
    {
        p_3d_origin.emplace_back( p_4d_origin.at<float>(0,i) / p_4d_origin.at<float>(3,i),
                                  p_4d_origin.at<float>(1,i) / p_4d_origin.at<float>(3,i),
                                  p_4d_origin.at<float>(2,i) / p_4d_origin.at<float>(3,i) );
    }

    this->origin_vec.emplace_back(p_3d_origin[1] - p_3d_origin[0]);
    this->origin_vec.emplace_back(p_3d_origin[2] - p_3d_origin[0]);
    this->origin_vec.emplace_back(this->origin_vec[0].cross(this->origin_vec[1]));

    /*
    Eigen::Matrix4d transformation;
    transformation <<   1.0,                                 v_3d_origin[1][0]/v_3d_origin[1][1], v_3d_origin[2][0]/v_3d_origin[2][2], - p_3d_origin[0][0],
                        v_3d_origin[0][1]/v_3d_origin[0][0], 1.0,                                 v_3d_origin[2][1]/v_3d_origin[2][2], - p_3d_origin[0][1],
                        v_3d_origin[0][2]/v_3d_origin[0][0], v_3d_origin[1][2]/v_3d_origin[1][1], 1.0,                                 - p_3d_origin[0][2],
                                      0.0,               0.0,               0.0,               1.0;
    Eigen::Vector4d obj;
    obj << p_3d_origin[1][0], p_3d_origin[1][1], p_3d_origin[1][2], 1.0;
     */
    /*
    Eigen::Vector3d coords_new, unit_x_new, unit_y_new, unit_z_new;

    unit_x_new << v_3d_origin[0][0]/v_3d_origin[0][0], v_3d_origin[0][1]/v_3d_origin[0][0], v_3d_origin[0][2]/v_3d_origin[0][0];
    unit_y_new << v_3d_origin[1][0]/v_3d_origin[1][1], v_3d_origin[1][1]/v_3d_origin[1][1], v_3d_origin[1][2]/v_3d_origin[1][1];
    unit_z_new << v_3d_origin[2][0]/v_3d_origin[2][2], v_3d_origin[2][1]/v_3d_origin[2][2], v_3d_origin[2][2]/v_3d_origin[2][2];

    coords_new[0] = (p_3d_origin[1] - p_3d_origin[0]).dot(unit_x_new);
    coords_new[1] = (p_3d_origin[1] - p_3d_origin[0]).dot(unit_y_new);
    coords_new[2] = (p_3d_origin[1] - p_3d_origin[0]).dot(unit_z_new);

    std::cout << "new transformation" << std::endl << coords_new << std::endl;
    */
    this->rotation_mtrx <<  this->origin_vec[0][0], this->origin_vec[1][0], this->origin_vec[2][0],
                            this->origin_vec[0][1], this->origin_vec[1][1], this->origin_vec[2][1],
                            this->origin_vec[0][2], this->origin_vec[1][2], this->origin_vec[2][2];

    this->origin = p_3d_origin[0];

    cv::aruco::drawDetectedMarkers(left_inp, left_markerCorners, left_markerIds);
    cv::aruco::drawDetectedMarkers(right_inp, right_markerCorners, right_markerIds);

    cv::Mat left_out(mono_size,CV_8UC4, stereoFrame->left_camera->rgba_buf.hImageBuf);
    cv::Mat right_out(mono_size, CV_8UC4, stereoFrame->right_camera->rgba_buf.hImageBuf);

    cv::cvtColor(left_inp, left_out, cv::COLOR_RGB2RGBA);
    cv::cvtColor(right_inp, right_out, cv::COLOR_RGB2RGBA);

    (*this->frame_manager)->sendFrame(stereoFrame);

    return NSSC_STATUS_SUCCESS;
}

std::vector<Eigen::Vector3d> nssc::process::Triangulation::_transformCoordinates(const std::vector<Eigen::Vector3d>& inp)
{
    Eigen::Vector3d obj_vec, mtrx_ret, obj;
    std::vector<Eigen::Vector3d> objects;

    for (auto & i : inp)
    {
        obj_vec = i - this->origin;
        mtrx_ret = this->rotation_mtrx.colPivHouseholderQr().solve(obj_vec);
        obj <<  (this->origin_vec[0] * mtrx_ret[0]).norm(),
                (this->origin_vec[1] * mtrx_ret[1]).norm(),
                (this->origin_vec[2] * mtrx_ret[2]).norm();
        objects.emplace_back(obj);
    }

    return objects;
}

Eigen::Vector3d nssc::process::Triangulation::_transformCoordinates(const Eigen::Vector3d &inp)
{
    Eigen::Vector3d obj_vec, mtrx_ret, obj;

    obj_vec = inp - this->origin;
    mtrx_ret = this->rotation_mtrx.colPivHouseholderQr().solve(obj_vec);
    obj <<  (this->origin_vec[0] * mtrx_ret[0]).norm(),
            (this->origin_vec[1] * mtrx_ret[1]).norm(),
            (this->origin_vec[2] * mtrx_ret[2]).norm();

    std::cout << mtrx_ret << std::endl;

    return obj;
}

bool nssc::process::Triangulation::__originMarkersExist(std::vector<int> ids)
{
    std::vector<int>& req_ids = this->node->g_config.triangulation_config.origin_ids;

    return std::all_of(req_ids.begin(), req_ids.end(), [&ids](int a_elt) {
        return std::find(ids.begin(), ids.end(), a_elt) != ids.end();
    });
}