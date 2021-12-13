#include "node.h"

NSSC::NSSC() : Node("NSSC") 
{
    RCLCPP_INFO(this->get_logger(), "[Node] initialized!");
}

void NSSC::printInfo(std::string caller, std::string msg)
{
    boost::format fmter("[%1%] %2%");
    fmter % caller; fmter % msg;
    RCLCPP_INFO(this->get_logger(), fmter.str());
}