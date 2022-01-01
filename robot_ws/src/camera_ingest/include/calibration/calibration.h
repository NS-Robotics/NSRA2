#ifndef NSSC_CALIBRATION_
#define NSSC_CALIBRATION_

#include "node.h"
#include "nssc_errors.h"

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sys/stat.h>
#include <boost/asio/io_service.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

struct StereoRepr
{
public:
    StereoRepr()
    {  
    }
    StereoRepr(std::vector<cv::Point2f> left_image_points, std::vector<cv::Point2f> right_image_points)
    {
        this->left_image_points = left_image_points;
        this->right_image_points = right_image_points;
    }
    std::vector<cv::Point2f> left_image_points;
    std::vector<cv::Point2f> right_image_points;
};

struct MonoRepr
{
public:
    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::vector<cv::Point3f>> object_points;
};

class Calibration : public NSSC_ERRORS
{
public:
    Calibration(std::shared_ptr<NSSC> &node, char *setName);
    ~Calibration();
    void cancel_calibration();

private:
    std::shared_ptr<NSSC> node;
    std::string msgCaller = "Calibration";

    void _prepareDataSet();
    void __CBCthreadTask(int img_num);
    std::tuple<NSSC_STATUS, std::vector<cv::Point2f>> __findCBC(char *in_file, char *out_file);
    void _calib_intrinsics();
    void _calib_stereo();
    void _undistort_rectify();
    bool __fileExists(const std::string &name);

    boost::shared_ptr<boost::asio::io_service> io_service_;
    boost::shared_ptr<boost::asio::io_service::work> work_;
    boost::thread_group threadpool_;

    std::string setPath;
    int board_width;
    int board_height;
    int num_images;
    cv::Size board_size;

    MonoRepr left_repr;
    MonoRepr right_repr;

    std::atomic<int> CBCThreads{0};

    std::vector<std::vector<cv::Point3f>> stereo_object_points;
    std::vector<StereoRepr> stereo_image_points;
    std::vector<std::vector<cv::Point2f>> stereo_left_image_points;
    std::vector<std::vector<cv::Point2f>> stereo_right_image_points;

    std::vector<cv::Point3f> obj;

    cv::Mat left_K, right_K, left_D, right_D;
    cv::Mat R, F, E;
    cv::Vec3d T;
    cv::Mat RL, RR, PL, PR, Q;
};

#endif //NSSC_CALIBRATION_