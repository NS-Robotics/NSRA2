#include "calibration.h"

void Calibration::_calib_intrinsics()
{
    std::vector<cv::Mat> rvecs, tvecs;
    int flag = 0;
}

double Calibration::__computeReprojectionErrors(const std::vector<ObjectRepr> &obj_repr, const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
                                              const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs)
{
    std::vector<cv::Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    std::vector<float> perViewErrors;
    perViewErrors.resize(obj_repr.size());

    for (i = 0; i < (int)obj_repr.size(); ++i)
    {
        cv::projectPoints(cv::Mat(obj_repr->object_points[i]), rvecs[i], tvecs[i], cameraMatrix,
                          distCoeffs, imagePoints2);
        err = norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), cv::L2);
        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err * err / n);
        totalErr += err * err;
        totalPoints += n;
    }
    return std::sqrt(totalErr / totalPoints);
}