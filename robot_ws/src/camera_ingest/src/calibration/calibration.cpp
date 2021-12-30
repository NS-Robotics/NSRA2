#include "calibration.h"

Calibration::Calibration(std::shared_ptr<NSSC> &node, char *setName) : NSSC_ERRORS(node)
{
    this->node = node;
    this->setPath = this->node->g_config.share_dir + "/" + setName + "/";
    this->board_width = this->node->g_config.calibConfig.board_width;
    this->board_height = this->node->g_config.calibConfig.board_height;
    _prepareDataSet();
}

void Calibration::_prepareDataSet()
{
    this->board_size = cv::Size(board_width, board_height);
    int board_n = board_width * board_height;

    for (int k = 0; k < this->board_height; k++)
        for (int j = 0; j < this->board_width; j++)
            this->obj.push_back(cv::Point3f((float)j * this->node->g_config.calibConfig.square_size, (float)k * this->node->g_config.calibConfig.square_size, 0));

    cv::FileStorage data_config((this->setPath + "config.xml").c_str(), cv::FileStorage::READ);
    if(!data_config.isOpened())
    {
        this->node->printWarning(this->msgCaller, "Config file failed to open");
        return;
    }
    data_config["ingestAmount"] >> this->num_images;

    for (int i = 0; i < this->num_images; i++)
    {
        NSSC_STATUS status;
        ObjectRepr ret;

        char img_file1[100];
        sprintf(img_file1, "%s%s%d.png", (this->setPath).c_str(), this->node->g_config.ingestConfig.right_img_name, i);
        std::string s_img_file1(img_file1);

        std::tie(status, ret) = __findCBC(img_file1);
        if(status == NSSC_CALIB_FILE_NOT_FOUND)
            this->node->printError(this->msgCaller, "File " + s_img_file1 + " not found");
        else if(status == NSSC_CALIB_CBC_NOT_FOUND)
            this->node->printWarning(this->msgCaller, "Right image " + std::to_string(i) + " CBC not found");
        else
            this->right_cam_repr.push_back(ret);

        char img_file2[100];
        sprintf(img_file2, "%s%s%d.png", (this->setPath).c_str(), this->node->g_config.ingestConfig.left_img_name, i);
        std::string s_img_file2(img_file2);

        std::tie(status, ret) = __findCBC(img_file2);
        if (status == NSSC_CALIB_FILE_NOT_FOUND)
            this->node->printError(this->msgCaller, "File " + s_img_file2 + " not found");
        else if (status == NSSC_CALIB_CBC_NOT_FOUND)
            this->node->printWarning(this->msgCaller, "Left image " + std::to_string(i) + " CBC not found");
        else
            this->left_cam_repr.push_back(ret);
    }
}

std::tuple<NSSC_STATUS, ObjectRepr> Calibration::__findCBC(char *img_file)
{
    ObjectRepr ret;
    cv::Mat img, gray;
    std::vector<cv::Point2f> corners;

    if (!__fileExists(img_file))
        return std::make_tuple(NSSC_CALIB_FILE_NOT_FOUND, ret);

    img = cv::imread(img_file, cv::IMREAD_COLOR);
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    bool found = false;
    found = cv::findChessboardCorners(img, this->board_size, corners,
                                      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
    
    if (!found)
        return std::make_tuple(NSSC_CALIB_CBC_NOT_FOUND, ret);

    cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
    cv::drawChessboardCorners(gray, this->board_size, corners, found);

    ret.object_points = this->obj;
    ret.image_points = corners;

    return std::make_tuple(NSSC_STATUS_SUCCESS, ret);
}

bool Calibration::__fileExists(const std::string &name)
{
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}