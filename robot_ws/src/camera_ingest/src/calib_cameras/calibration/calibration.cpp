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

#include "calibration.h"
#include "node.h"
#include "nssc_errors.h"

nssc::stereocalibration::Calibration::Calibration(std::shared_ptr<nssc::ros::NSSC> &node, char *setName) : NSSC_ERRORS(node)
{
    this->node = node;
    this->set_path = this->node->g_config.share_dir + "/" + setName + "/";
    this->board_width = this->node->g_config.calibConfig.board_width;
    this->board_height = this->node->g_config.calibConfig.board_height;

    this->_io_service = boost::make_shared<boost::asio::io_service>();
    this->_work = boost::make_shared<boost::asio::io_service::work>(*this->_io_service);
    for(int i = 0; i < 10; i++)
    {
        this->_threadpool.create_thread(boost::bind(&boost::asio::io_service::run, this->_io_service));
    }

    this->node->printInfo(this->msg_caller, "prepareDataSet");
    _prepareDataSet();
    _calibIntrinsics();
    _calibStereo();
    _saveConfig();
}

nssc::stereocalibration::Calibration::~Calibration()
{

}

void nssc::stereocalibration::Calibration::_calibIntrinsics()
{
    std::vector<cv::Mat> rvecs, tvecs;
    int flag = 0;
    flag |= cv::CALIB_FIX_K4;
    flag |= cv::CALIB_FIX_K5;

    this->rms_left = cv::calibrateCamera(left_repr.object_points, left_repr.image_points, cv::Size(3088, 2064), this->left_K, this->left_D, rvecs, tvecs, flag);

    this->node->printInfo(this->msg_caller, "Left camera intrinsics reprojection error: " + std::to_string(rms_left));

    this->rms_right = cv::calibrateCamera(right_repr.object_points, right_repr.image_points, cv::Size(3088, 2064), this->right_K, this->right_D, rvecs, tvecs, flag);

    this->node->printInfo(this->msg_caller, "Right camera intrinsics reprojection error: " + std::to_string(rms_right));
}

void nssc::stereocalibration::Calibration::_calibStereo()
{
    int flag = 0;
    flag |= cv::CALIB_FIX_INTRINSIC;

    this->rms_stereo = cv::stereoCalibrate(this->stereo_object_points, this->stereo_left_image_points, this->stereo_right_image_points,
                                     this->left_K, this->left_D, this->right_K, this->right_D, cv::Size(3088, 2064), this->R, this->T, this->E, this->F);

    this->node->printInfo(this->msg_caller, "Stereo calibration reprojection error: " + std::to_string(rms_stereo));

    cv::stereoRectify(this->left_K, this->left_D, this->right_K, this->right_D, cv::Size(3088, 2064),
                      this->R, this->T, this->RL, this->RR, this->PL, this->PR, this->Q);

    this->node->printInfo(this->msg_caller, "Stereo rectification complete");
}

void nssc::stereocalibration::Calibration::_saveConfig()
{
    cv::FileStorage config_file((this->set_path + "config.xml").c_str(), cv::FileStorage::WRITE);
    //rms
    config_file << "rms_left" << this->rms_left;
    config_file << "rms_right" << this->rms_right;
    config_file << "rms_stereo" << this->rms_stereo;
    //intrinsics
    config_file << "left_K" << this->left_K;
    config_file << "left_D" << this->left_D;
    config_file << "right_K" << this->right_K;
    config_file << "right_D" << this->right_D;
    //stereo
    config_file << "R" << this->R;
    config_file << "T" << this->T;
    config_file << "E" << this->E;
    config_file << "F" << this->F;
    //rectification
    config_file << "left_R" << this->RL;
    config_file << "right_R" << this->RR;
    config_file << "left_P" << this->PL;
    config_file << "right_P" << this->PR;
    config_file << "Q" << this->Q;

    this->node->printInfo(this->msg_caller, "Configuration saved");
}