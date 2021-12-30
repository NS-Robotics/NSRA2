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

class Calibration
{
public:
    Calibration(std::shared_ptr<NSSC> &node, char *setName);
    void cancel_calibration();

private:
    std::shared_ptr<NSSC> node;
    std::string msgCaller = "Calibration";

    char* setPath;
    int board_width;
    int board_height;
    int num_images;
    cv::Size board_size;

    std::vector<ObjectRepr> left_cam_repr, right_cam_repr;

    void _prepareDataSet();
    std::tuple<bool, ObjectRepr> __findCBC(char *fileName);
    void _calib_intrinsics();
    void _calib_stereo();
    void _undistort_rectify();
    bool __fileExists(const char* fileName);
};

#endif //NSSC_CALIBRATION_