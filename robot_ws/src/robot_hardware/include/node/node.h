#pragma once

#include "rclcpp/rclcpp.hpp"
#include <boost/format.hpp>
#include "robot_descriptions/msg/init.hpp"
#include "robot_descriptions/msg/pos.hpp"
#include "robot_descriptions/msg/vel.hpp"
#include "robot_descriptions/msg/stop.hpp"

using std::placeholders::_1;

class RobotSystemPublisher : public rclcpp::Node
{
public:
    RobotSystemPublisher();
    void printInfo(std::string caller, std::string msg);
    void printError(std::string caller, std::string msg);

private:
    const std::string caller = "Node";
    rclcpp::Publisher<robot_descriptions::msg::Pos>::SharedPtr publisher_;
};