#include "image_transport.h"
#include <chrono> 

NSSCImageTransport::NSSCImageTransport(std::shared_ptr<NSSC>& node, std::shared_ptr<cameraManager>& camManager) : NSSC_ERRORS(node)
{
    this->camManager = camManager;
    this->node = node;

    this->left_pub = image_transport::create_publisher(this->node.get(), "NSSC_left_camera", this->custom_qos);
    this->right_pub = image_transport::create_publisher(this->node.get(), "NSSC_right_camera", this->custom_qos);
}

NSSCImageTransport::~NSSCImageTransport()
{
    this->streamON = false;
    RCLCPP_INFO(this->node->get_logger(), "transport destructor called!");
}

NSSC_STATUS NSSCImageTransport::startPipeline()
{

    NSSC_STATUS status = NSSC_STATUS_SUCCESS;

    this->streamON = true;

    this->sThread = std::thread(&NSSCImageTransport::transThreadFunc, this);

    RCLCPP_INFO(this->node->get_logger(), "transport Thread started!");

    return status;
}

void NSSCImageTransport::transThreadFunc()
{
    RCLCPP_INFO(this->node->get_logger(), "Hello from the transport Thread!");

    while(this->streamON.load())
	{	

        auto start1 = std::chrono::high_resolution_clock::now();

        auto [g_pRGBImageBufLeft, g_pRGBImageBufRight] = this->camManager->getFrame();

        auto stop1 = std::chrono::high_resolution_clock::now();

        auto start2 = std::chrono::high_resolution_clock::now();

        cv::Mat leftMat(2064, 3088, CV_8UC3, g_pRGBImageBufLeft);
        //cv::Mat leftScaled;
        //cv::resize(leftMat, leftScaled, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);

        sensor_msgs::msg::Image::SharedPtr leftMsg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", leftMat).toImageMsg();
        this->left_pub.publish(leftMsg);

        cv::Mat rightMat(3088, 2064, CV_8UC3, g_pRGBImageBufRight);
        //cv::Mat rightScaled;
        //cv::resize(rightMat, rightScaled, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);

        sensor_msgs::msg::Image::SharedPtr rightMsg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rightMat).toImageMsg();
        this->right_pub.publish(rightMsg);

        delete[] g_pRGBImageBufLeft;
        g_pRGBImageBufLeft = NULL;

        delete[] g_pRGBImageBufRight;
        g_pRGBImageBufRight = NULL;

        auto stop2 = std::chrono::high_resolution_clock::now();
        
        auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(stop1 - start1);
        auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(stop2 - start2);

        RCLCPP_INFO(this->node->get_logger(), "Frame timing - getFrame: " + std::to_string(duration1.count()) + " send: " + std::to_string(duration2.count()));
	}
}