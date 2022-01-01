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
#include <boost/make_shared.hpp>

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

private:
    std::shared_ptr<NSSC> node;
    std::string msgCaller = "Calibration";

    void _prepareDataSet();
    void __CBCthreadTask(int img_num);
    std::tuple<NSSC_STATUS, std::vector<cv::Point2f>> __findCBC(char *in_file, char *out_file);
    void _calibIntrinsics();
    void _calibStereo();
    void _undistort_rectify();
    bool __fileExists(const std::string &name);
    void _saveConfig();

    boost::shared_ptr<boost::asio::io_service> _io_service;
    boost::shared_ptr<boost::asio::io_service::work> _work;
    boost::thread_group _threadpool;

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

    double rms_left;
    double rms_right;
    double rms_stereo;
};

#endif //NSSC_CALIBRATION_