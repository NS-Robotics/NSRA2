#include "calibration.h"
#include "nssc_errors.h"

void nssc::stereocalibration::Calibration::_prepareDataSet()
{
    this->board_size = cv::Size(board_width, board_height);
    int board_n = board_width * board_height;

    for (int k = 0; k < this->board_height; k++)
        for (int j = 0; j < this->board_width; j++)
            this->obj.push_back(cv::Point3f((float)j * this->node->g_config.calib_config.square_size, (float)k * this->node->g_config.calib_config.square_size, 0));

    cv::FileStorage data_config((this->set_path + "config.xml").c_str(), cv::FileStorage::READ);
    if (!data_config.isOpened())
    {
        this->node->printWarning(this->msg_caller, "Config file not found");
        return;
    }
    data_config["ingestAmount"] >> this->num_images;

    for (int i = 0; i < this->num_images; i++)
    {
        this->_io_service->post([this, i] { __CBCthreadTask(i); });
    }

    while(this->cbc_threads.load() != this->num_images)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    this->_io_service->stop();
    this->_threadpool.join_all();
    this->node->printInfo(this->msg_caller, "Image processing complete");

    for (int i = 0; i < this->stereo_image_points.size(); i++)
    {
        std::vector<cv::Point2f> n_left, n_right;
        for (int j = 0; j < this->stereo_image_points[i].left_image_points.size(); j++)
        {
            n_left.push_back(cv::Point2f((double)this->stereo_image_points[i].left_image_points[j].x, (double)this->stereo_image_points[i].left_image_points[j].y));
            n_right.push_back(cv::Point2f((double)this->stereo_image_points[i].right_image_points[j].x, (double)this->stereo_image_points[i].right_image_points[j].y));
        }
        this->stereo_left_image_points.push_back(n_left);
        this->stereo_right_image_points.push_back(n_right);
    }
}

void nssc::stereocalibration::Calibration::__CBCthreadTask(int img_num)
{
    nssc::stereocalibration::StereoRepr ret;

    NSSC_STATUS right_status;
    std::vector<cv::Point2f> right_img_points;
    char in_file1[100];
    char out_file1[100];
    sprintf(in_file1, "%s%s%d.png", (this->set_path).c_str(), this->node->g_config.ingest_config.right_img_name, img_num);
    sprintf(out_file1, "%s%s%s%d.png", (this->set_path).c_str(), "corners_", this->node->g_config.ingest_config.right_img_name, img_num);
    std::string s_img_file1(in_file1);

    std::tie(right_status, right_img_points) = __findCBC(in_file1, out_file1);
    if (right_status == NSSC_CALIB_FILE_NOT_FOUND)
        this->node->printError(this->msg_caller, "File " + s_img_file1 + " not found");
    else if (right_status == NSSC_CALIB_CBC_NOT_FOUND)
        this->node->printWarning(this->msg_caller, "Right image " + std::to_string(img_num) + " CBC not found");
    else
    {
        this->right_repr.image_points.push_back(right_img_points);
        this->right_repr.object_points.push_back(this->obj);
        this->node->printInfo(this->msg_caller, "Right image " + std::to_string(img_num) + " CBC detection successfull");
    }

    NSSC_STATUS left_status;
    std::vector<cv::Point2f> left_img_points;
    char in_file2[100];
    char out_file2[100];
    sprintf(in_file2, "%s%s%d.png", (this->set_path).c_str(), this->node->g_config.ingest_config.left_img_name, img_num);
    sprintf(out_file2, "%s%s%s%d.png", (this->set_path).c_str(), "corners_", this->node->g_config.ingest_config.left_img_name, img_num);
    std::string s_img_file2(in_file2);

    std::tie(left_status, left_img_points) = __findCBC(in_file2, out_file2);
    if (left_status == NSSC_CALIB_FILE_NOT_FOUND)
        this->node->printError(this->msg_caller, "File " + s_img_file2 + " not found");
    else if (left_status == NSSC_CALIB_CBC_NOT_FOUND)
        this->node->printWarning(this->msg_caller, "Left image " + std::to_string(img_num) + " CBC not found");
    else
    {
        this->left_repr.image_points.push_back(left_img_points);
        this->left_repr.object_points.push_back(this->obj);
        this->node->printInfo(this->msg_caller, "Left image " + std::to_string(img_num) + " CBC detection successfull");
    }

    if (right_status == NSSC_STATUS_SUCCESS && left_status == NSSC_STATUS_SUCCESS)
    {
        nssc::stereocalibration::StereoRepr ret(left_img_points, right_img_points);
        this->stereo_image_points.push_back(ret);
        this->stereo_object_points.push_back(this->obj);
    }

    this->cbc_threads++;
}

std::tuple<nssc::NSSC_STATUS, std::vector<cv::Point2f>> nssc::stereocalibration::Calibration::__findCBC(char *in_file, char *out_file)
{
    cv::Mat img, gray;
    std::vector<cv::Point2f> corners;

    if (!__fileExists(in_file))
        return std::make_tuple(NSSC_CALIB_FILE_NOT_FOUND, corners);

    img = cv::imread(in_file, cv::IMREAD_COLOR);
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    bool found = false;
    found = cv::findChessboardCorners(img, this->board_size, corners,
                                      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);

    if (!found)
        return std::make_tuple(NSSC_CALIB_CBC_NOT_FOUND, corners);

    cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
    cv::drawChessboardCorners(img, this->board_size, corners, found);
    /*
    std::string file_name(out_file);
    cv::imwrite(file_name, img);
    */
    return std::make_tuple(NSSC_STATUS_SUCCESS, corners);
}

bool nssc::stereocalibration::Calibration::__fileExists(const std::string &name)
{
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}