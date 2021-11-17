#include "node.h"

RobotSystemPublisher::RobotSystemPublisher() : Node("NSRA2_EMU") 
{
    publisher_ = this->create_publisher<robot_descriptions::msg::Pos>("pos_action", 10);

    RCLCPP_INFO(this->get_logger(), "[Node] initialized!");
}

void RobotSystemPublisher::printInfo(std::string caller, std::string msg)
{
    boost::format fmter("[%1%] %2%");
    fmter % caller; fmter % msg;
    RCLCPP_INFO(this->get_logger(), fmter.str());
}

void RobotSystemPublisher::printError(std::string caller, std::string msg)
{
    boost::format fmter("[%1%] %2%");
    fmter % caller; fmter % msg;
    RCLCPP_ERROR(this->get_logger(), fmter.str());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotSystemPublisher>();
    std::thread spinner(&spin);

    rclcpp::spin(node);
    rclcpp::shutdown();
}