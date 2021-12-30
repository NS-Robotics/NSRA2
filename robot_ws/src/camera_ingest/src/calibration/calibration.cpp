#include "calibration.h"

Calibration::Calibration(std::shared_ptr<NSSC> &node, char *setName) : NSSC_ERRORS(node)
{
    this->node = node;
    this->setPath = this->node->g_config.share_dir + "/" + setName + "/";
    this->board_width = this->node->g_config.calibConfig.board_width;
    this->board_height = this->node->g_config.calibConfig.board_height;
    _prepareDataSet();
    _calib_intrinsics();
    _calib_stereo();
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
        this->node->printWarning(this->msgCaller, "Config file not found");
        return;
    }
    data_config["ingestAmount"] >> this->num_images;

    for (int i = 0; i < this->num_images; i++)
    {
        NSSC_STATUS status;
        ObjectRepr ret;

        char in_file1[100];
        char out_file1[100];
        sprintf(in_file1, "%s%s%d.png", (this->setPath).c_str(), this->node->g_config.ingestConfig.right_img_name, i);
        sprintf(out_file1, "%s%s%s%d.png", (this->setPath).c_str(), "corners_", this->node->g_config.ingestConfig.right_img_name, i);
        std::string s_img_file1(in_file1);

        std::tie(status, ret) = __findCBC(in_file1, out_file1);
        if(status == NSSC_CALIB_FILE_NOT_FOUND)
            this->node->printError(this->msgCaller, "File " + s_img_file1 + " not found");
        else if(status == NSSC_CALIB_CBC_NOT_FOUND)
            this->node->printWarning(this->msgCaller, "Right image " + std::to_string(i) + " CBC not found");
        else
        {
            this->right_cam_repr.object_points.push_back(ret.object_points);
            this->right_cam_repr.image_points.push_back(ret.image_points);
        }

        char in_file2[100];
        char out_file2[100];
        sprintf(in_file2, "%s%s%d.png", (this->setPath).c_str(), this->node->g_config.ingestConfig.left_img_name, i);
        sprintf(out_file2, "%s%s%s%d.png", (this->setPath).c_str(), "corners_", this->node->g_config.ingestConfig.left_img_name, i);
        std::string s_img_file2(in_file2);

        std::tie(status, ret) = __findCBC(in_file2, out_file2);
        if (status == NSSC_CALIB_FILE_NOT_FOUND)
            this->node->printError(this->msgCaller, "File " + s_img_file2 + " not found");
        else if (status == NSSC_CALIB_CBC_NOT_FOUND)
            this->node->printWarning(this->msgCaller, "Left image " + std::to_string(i) + " CBC not found");
        else
        {
            this->left_cam_repr.object_points.push_back(ret.object_points);
            this->left_cam_repr.image_points.push_back(ret.image_points);
        }
    }

    for (int i = 0; i < this->left_cam_repr.image_points.size(); i++)
    {
        std::vector<cv::Point2f> v1, v2;
        for (int j = 0; j < this->left_cam_repr.image_points[i].size(); j++)
        {
            v1.push_back(cv::Point2f((double)this->left_cam_repr.image_points[i][j].x, (double)this->left_cam_repr.image_points[i][j].y));
            v2.push_back(cv::Point2f((double)this->right_cam_repr.image_points[i][j].x, (double)this->right_cam_repr.image_points[i][j].y));
        }
        this->left_img_repr.image_points.push_back(v1);
        this->left_img_repr.object_points.push_back(this->left_cam_repr.object_points[i]);
        this->right_img_repr.image_points.push_back(v2);
        this->right_img_repr.object_points.push_back(this->right_cam_repr.object_points[i]);
    }
}

std::tuple<NSSC_STATUS, ObjectRepr> Calibration::__findCBC(char *in_file, char *out_file)
{
    ObjectRepr ret;
    cv::Mat img, gray;
    std::vector<cv::Point2f> corners;

    if (!__fileExists(in_file))
        return std::make_tuple(NSSC_CALIB_FILE_NOT_FOUND, ret);

    img = cv::imread(in_file, cv::IMREAD_COLOR);
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    bool found = false;
    found = cv::findChessboardCorners(img, this->board_size, corners,
                                      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
    
    if (!found)
        return std::make_tuple(NSSC_CALIB_CBC_NOT_FOUND, ret);

    cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
    cv::drawChessboardCorners(img, this->board_size, corners, found);

    std::string file_name(out_file);
    cv::imwrite(file_name, img);

    ret.object_points = this->obj;
    ret.image_points = corners;

    return std::make_tuple(NSSC_STATUS_SUCCESS, ret);
}

bool Calibration::__fileExists(const std::string &name)
{
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}