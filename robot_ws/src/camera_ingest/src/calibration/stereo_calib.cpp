#include "calibration.h"

void Calibration::_calib_stereo()
{
    int flag = 0;
    flag |= cv::CALIB_FIX_INTRINSIC;

    double rms = cv::stereoCalibrate(this->stereo_object_points, this->stereo_left_image_points, this->stereo_right_image_points,
                                     this->left_K, this->left_D, this->right_K, this->right_D, cv::Size(3088, 2064), this->R, this->T, this->E, this->F);

    this->node->printInfo(this->msgCaller, "Stereo calibration reprojection error: " + std::to_string(rms));

    cv::stereoRectify(this->left_K, this->left_D, this->right_K, this->right_D, cv::Size(3088, 2064),
                      this->R, this->T, this->RL, this->RR, this->PL, this->PR, this->Q);
}