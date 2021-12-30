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
    Mat img, gray;
    std::vector<cv::Point2f> corners;
    std::vector<cv::Point3f> obj;

    this->board_size = Size(board_width, board_height);
    int board_n = board_width * board_height;

    for (int k = 0; k < this->board_height; k++)
        for (int j = 0; j < this->board_width; j++)
            obj.push_back(Point3f((float)j * this->node->g_config.calibConfig.square_size, (float)k * this->node->g_config.calibConfig.square_size, 0));

    cv::FileStorage data_config(this->setPath + "config.xml", FileStorage::READ);
    if(!data_config.isOpened())
    {
        this->node->printWarning(this->msgCaller, "Config file failed to open");
        return;
    }
    data_config["ingestAmount"] >> this->num_images;

    for (int i = 0; i < num_images; i++)
    {

    }
}

std::tuple<bool, ObjectRepr> Calibration::__findCBC(char *fileName)
{
    ObjectRepr ret;

    char *img_file;
    sprintf(img_file, "%s%s%d.jpg", this->setPath, fileName, i);

    if (!__fileExists(img_file))
        this->node->printWarning(this->msgCaller, "Image " + fileName + std::to_string(i) + " not found!");
    continue;

    img = cv::imread(img_file, cv::CV_LOAD_IMAGE_COLOR);
    cv::cvtColor(img, gray, cv::CV_BGR2GRAY);

    bool found = false;
    found = cv::findChessboardCorners(img, this->board_size, corners,
                                      cv::CV_CALIB_CB_ADAPTIVE_THRESH | cv::CV_CALIB_CB_FILTER_QUADS);

    if (!found)
    {
        this->node->printWarning(this->msgCaller, "Image " + fileName + std::to_string(i) + " chessboard not found!");
        return std::make_tuple(found, ret);
    }

    cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                     cv::TermCriteria(cv::CV_TERMCRIT_EPS | cv::CV_TERMCRIT_ITER, 30, 0.1));
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