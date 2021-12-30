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
        this->ioService.post(boost::bind(__CBCthreadTask, i));
    }

    this->ioService.stop();
    this->threadpool.join_all();

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

void Calibration::__CBCthreadTask(int img_num)
{
    StereoRepr ret;

    this->node->printInfo(this->msgCaller, "Thread image task: " + std::to_string(img_num));

    NSSC_STATUS right_status;
    std::vector<cv::Point2f> right_img_points;
    char in_file1[100];
    char out_file1[100];
    sprintf(in_file1, "%s%s%d.png", (this->setPath).c_str(), this->node->g_config.ingestConfig.right_img_name, img_num);
    sprintf(out_file1, "%s%s%s%d.png", (this->setPath).c_str(), "corners_", this->node->g_config.ingestConfig.right_img_name, img_num);
    std::string s_img_file1(in_file1);

    std::tie(right_status, right_img_points) = __findCBC(in_file1, out_file1);
    if (right_status == NSSC_CALIB_FILE_NOT_FOUND)
        this->node->printError(this->msgCaller, "File " + s_img_file1 + " not found");
    else if (right_status == NSSC_CALIB_CBC_NOT_FOUND)
        this->node->printWarning(this->msgCaller, "Right image " + std::to_string(img_num) + " CBC not found");
    else
    {
        this->right_repr.image_points.push_back(right_img_points);
        this->right_repr.object_points.push_back(this->obj);
    } 

    NSSC_STATUS left_status;
    std::vector<cv::Point2f> left_img_points;
    char in_file2[100];
    char out_file2[100];
    sprintf(in_file2, "%s%s%d.png", (this->setPath).c_str(), this->node->g_config.ingestConfig.left_img_name, img_num);
    sprintf(out_file2, "%s%s%s%d.png", (this->setPath).c_str(), "corners_", this->node->g_config.ingestConfig.left_img_name, img_num);
    std::string s_img_file2(in_file2);

    std::tie(left_status, left_img_points) = __findCBC(in_file2, out_file2);
    if (left_status == NSSC_CALIB_FILE_NOT_FOUND)
        this->node->printError(this->msgCaller, "File " + s_img_file2 + " not found");
    else if (left_status == NSSC_CALIB_CBC_NOT_FOUND)
        this->node->printWarning(this->msgCaller, "Left image " + std::to_string(img_num) + " CBC not found");
    else
    {
        this->left_repr.image_points.push_back(right_img_points);
        this->left_repr.object_points.push_back(this->obj);
    }

    if(right_status == NSSC_STATUS_SUCCESS && left_status == NSSC_STATUS_SUCCESS)
    {   
        StereoRepr ret(left_img_points, right_img_points);
        this->stereo_image_points.push_back(ret);
        this->stereo_object_points.push_back(this->obj);
    }
}

std::tuple<NSSC_STATUS, std::vector<cv::Point2f>> Calibration::__findCBC(char *in_file, char *out_file)
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

    std::string file_name(out_file);
    cv::imwrite(file_name, img);

    return std::make_tuple(NSSC_STATUS_SUCCESS, corners);
}

bool Calibration::__fileExists(const std::string &name)
{
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}