#include "calibration.h"

Calibration::Calibration(std::shared_ptr<NSSC> &node, char *setName) : NSSC_ERRORS(node)
{
    this->node = node;
    this->setPath = this->node->g_config.share_dir + "/" + setName + "/";
    this->board_width = this->node->g_config.calibConfig.board_width;
    this->board_height = this->node->g_config.calibConfig.board_height;

    boost::asio::io_service::work work(this->ioService);
    for(int i = 0; i < 10; i++)
    {
        this->threadpool.create_thread(boost::bind(&boost::asio::io_service::run, &this->ioService));
    }
    _prepareDataSet();
    _calib_intrinsics();
    _calib_stereo();
}

Calibration::~Calibration()
{
    this->ioService.stop();
    this->threadpool.join_all();
}

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

void Calibration::_calib_stereo()
{
    int flag = 0;
    flag |= cv::CALIB_FIX_INTRINSIC;

    double rms = cv::stereoCalibrate(this->stereo_object_points, this->stereo_left_image_points, this->stereo_right_image_points,
                                     this->left_K, this->left_D, this->right_K, this->right_D, cv::Size(3088, 2064), this->R, this->T, this->E, this->F);

    this->node->printInfo(this->msgCaller, "Stereo calibration reprojection error: " + std::to_string(rms));

    cv::stereoRectify(this->left_K, this->left_D, this->right_K, this->right_D, cv::Size(3088, 2064),
                      this->R, this->T, this->RL, this->RR, this->PL, this->PR, this->Q);

    this->node->printInfo(this->msgCaller, "Stereo rectification complete");
}