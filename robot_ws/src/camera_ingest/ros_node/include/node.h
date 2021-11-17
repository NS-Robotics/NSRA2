#pragma once

#include "rclcpp/rclcpp.hpp"
#include <boost/format.hpp>

class NSSC : public rclcpp::Node
{
public:
    NSSC();
    void printInfo(std::string caller, std::string msg);
private:
};