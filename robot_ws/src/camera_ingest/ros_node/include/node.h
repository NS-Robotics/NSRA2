#pragma once

#include "rclcpp/rclcpp.hpp"
#include <boost/format.hpp>

#include "cli.h"

class NSSC : public rclcpp::Node
{
public:
    NSSC();
    ~NSSC();
    void printInfo(std::string caller, std::string msg);
    void openCLI();
    void closeCLI();

private:
    std::unique_ptr<CLI> cli;
};