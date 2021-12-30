#include "calibration.h"

void Calibration::_calib_intrinsics()
{
    std::vector<cv::Mat> rvecs, tvecs;
    int flag = 0;
    flag |= cv::CALIB_FIX_K4;
    flag |= cv::CALIB_FIX_K5;

    double rms_l = cv::calibrateCamera(left_repr.object_points, left_repr.image_points, cv::Size(3088, 2064), this->left_K, this->left_D, rvecs, tvecs, flag);

    this->node->printInfo(this->msgCaller, "Left camera intrinsics reprojection error: " + std::to_string(rms_l));

    double rms_r = cv::calibrateCamera(right_repr.object_points, right_repr.image_points, cv::Size(3088, 2064), this->right_K, this->right_D, rvecs, tvecs, flag);

    this->node->printInfo(this->msgCaller, "Right camera intrinsics reprojection error: " + std::to_string(rms_r));
}