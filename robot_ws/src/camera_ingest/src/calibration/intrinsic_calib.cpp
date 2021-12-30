#include "calibration.h"

void Calibration::_calib_intrinsics()
{
    std::vector<cv::Mat> rvecs, tvecs;
    int flag = 0;
    flag |= cv::CALIB_FIX_K4;
    flag |= cv::CALIB_FIX_K5;

    cv::calibrateCamera(left_cam_repr.object_points, left_cam_repr.image_points, cv::Size(3088, 2064), this->left_K, this->left_D, rvecs, tvecs, flag);
}

double Calibration::__computeReprojectionErrors(const ObjectReprVec &obj_repr, const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
                                                const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs)
{
    std::vector<cv::Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    std::vector<float> perViewErrors;
    perViewErrors.resize(obj_repr.object_points.size());

    for (i = 0; i < (int)obj_repr.object_points.size(); ++i)
    {
        cv::projectPoints(cv::Mat(obj_repr.object_points[i]), rvecs[i], tvecs[i], cameraMatrix,
                          distCoeffs, imagePoints2);
        err = norm(cv::Mat(obj_repr.image_points[i]), cv::Mat(imagePoints2), cv::NORM_L2);
        int n = (int)obj_repr.object_points[i].size();
        perViewErrors[i] = (float)std::sqrt(err * err / n);
        totalErr += err * err;
        totalPoints += n;
    }
    return std::sqrt(totalErr / totalPoints);
}