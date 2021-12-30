#include "node.h"

NSSC::NSSC() : Node("NSSC") 
{
    printInfo(this->msgCaller, "initialized!");
}

void NSSC::printInfo(std::string caller, std::string msg)
{
    boost::format fmter("\033[1;34m[%1%] \033[0m%2%");
    fmter % caller; fmter % msg;
    RCLCPP_INFO(this->get_logger(), fmter.str());
}

void NSSC::printWarning(std::string caller, std::string msg)
{
    boost::format fmter("\033[1;34m[%1%] \033[0m%2%");
    fmter % caller; fmter % msg;
    RCLCPP_WARN(this->get_logger(), fmter.str());
}

void NSSC::printError(std::string caller, std::string msg)
{
    boost::format fmter("\033[1;34m[%1%] \033[0m%2%");
    fmter % caller;
    fmter % msg;
    RCLCPP_ERROR(this->get_logger(), fmter.str());
}

void NSSC::printFatal(std::string caller, std::string msg)
{
    boost::format fmter("\033[1;34m[%1%] \033[0m%2%");
    fmter % caller;
    fmter % msg;
    RCLCPP_FATAL(this->get_logger(), fmter.str());
}