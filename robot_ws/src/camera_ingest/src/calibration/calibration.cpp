#include "calibration.h"

Calibration::Calibration(std::shared_ptr<NSSC> &node, char *setName)
{
    this->node = node;
    this->setPath = this->node->g_config.share_dir + "/" + setName + "/";
    this->board_width = this->node->g_config.calibConfig.board_width;
    this->board_height = this->node->g_config.calibConfig.board_height
    _prepareDataSet();
}

void Calibration::_prepareDataSet()
{
    this->board_size = cv::Size(board_width, board_height);
    int board_n = board_width * board_height;

    for (int k = 0; k < this->board_height; k++)
        for (int j = 0; j < this->board_width; j++)
            this->obj.push_back(cv::Point3f((float)j * this->node->g_config.calibConfig.square_size, (float)k * this->node->g_config.calibConfig.square_size, 0));

    cv::FileStorage data_config(this->setPath + "config.xml", cv::FileStorage::READ);
    if(!data_config.isOpened())
    {
        this->node->printWarning(this->msgCaller, "Config file failed to open");
        return;
    }
    data_config["ingestAmount"] >> this->num_images;

    for (int i = 0; i < num_images; i++)
    {
        char *img_file;
        sprintf(img_file, "%s%s%d.jpg", this->setPath, "fileName", i);
    }
}

std::tuple<bool, ObjectRepr> Calibration::__findCBC(char *img_file)
{
    ObjectRepr ret;
    cv::Mat img, gray;
    std::vector<cv::Point2f> corners;

    if (!__fileExists(img_file))
        this->node->printWarning(this->msgCaller, "Image " + img_file + " not found!");
    continue;

    img = cv::imread(img_file, cv::IMREAD_COLOR);
    cv::cvtColor(img, gray, cv::CV_BGR2GRAY);

    bool found = false;
    found = cv::findChessboardCorners(img, this->board_size, corners,
                                      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);

    if (!found)
    {
        this->node->printWarning(this->msgCaller, "Image " + img_file + " chessboard not found!");
        return std::make_tuple(found, ret);
    }

    cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
    cv::drawChessboardCorners(gray, this->board_size, corners, found);

    ret.object_points = this->obj;
    ret.image_points = corners;

    return std::make_tuple(found, ret);
}

bool Calibration::__fileExists(const char *fileName)
{
    struct stat buffer;
    return (stat(fileName, &buffer) == 0);
}