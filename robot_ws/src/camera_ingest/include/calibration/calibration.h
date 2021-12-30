#ifndef NSSC_CALIBRATION_
#define NSSC_CALIBRATION_

#include "node.h"
#include "nssc_errors.h"

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sys/stat.h>

struct ObjectRepr
{
public:
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
};

struct ObjectReprVec
{
public:
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
};

class Calibration : public NSSC_ERRORS
{
public:
    Calibration(std::shared_ptr<NSSC> &node, char *setName);
    void cancel_calibration();

private:
    std::shared_ptr<NSSC> node;
    std::string msgCaller = "Calibration";

    void _prepareDataSet();
    std::tuple<NSSC_STATUS, ObjectRepr> __findCBC(char *in_file, char *out_file);
    void _calib_intrinsics();
    void _calib_stereo();
    void _undistort_rectify();
    bool __fileExists(const std::string &name);
    double __computeReprojectionErrors(const ObjectReprVec &obj_repr,
                                       const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
                                       const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs);

    std::string setPath;
    int board_width;
    int board_height;
    int num_images;
    cv::Size board_size;

    ObjectReprVec left_cam_repr, right_cam_repr;
    ObjectReprVec left_img_repr, right_img_repr;
    std::vector<cv::Point3f> obj;

    cv::Mat left_K, right_K, left_D, right_D;
    cv::Mat R, F, E;
    cv::Vec3d T;
    cv::Mat RL, RR, PL, PR, Q;
};

#endif //NSSC_CALIBRATION_