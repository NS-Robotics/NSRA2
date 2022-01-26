#include "calibration.h"

Calibration::Calibration(std::shared_ptr<NSSC> &node, char *setName) : NSSC_ERRORS(node)
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

    _prepareDataSet();
    _calibIntrinsics();
    _calibStereo();
    _saveConfig();
}

Calibration::~Calibration()
{

}

void Calibration::_calibIntrinsics()
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

void Calibration::_calibStereo()
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

void Calibration::_saveConfig()
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
    config_file << "RL" << this->RL;
    config_file << "RR" << this->RR;
    config_file << "PL" << this->PL;
    config_file << "PR" << this->PR;
    config_file << "Q" << this->Q;

    this->node->printInfo(this->msg_caller, "Configuration saved");
}