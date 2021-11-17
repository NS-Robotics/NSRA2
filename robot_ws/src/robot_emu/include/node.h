#pragma once

#include "rclcpp/rclcpp.hpp"
#include <boost/format.hpp>
#include "robot_descriptions/msg/init.hpp"
#include "robot_descriptions/msg/pos.hpp"
#include "robot_descriptions/msg/vel.hpp"
#include "robot_descriptions/msg/stop.hpp"

using std::placeholders::_1;

class NSSC : public rclcpp::Node
{
public:
    NSSC();
    void printInfo(std::string caller, std::string msg);
    void printError(std::string caller, std::string msg);
    void init_callback(const robot_descriptions::msg::Init::SharedPtr msg); 
    void pos_callback(const robot_descriptions::msg::Pos::SharedPtr msg); 
    void vel_callback(const robot_descriptions::msg::Vel::SharedPtr msg); 
    void stop_callback(const robot_descriptions::msg::Stop::SharedPtr msg); 
private:
    const std::string caller = "Node";
    rclcpp::Subscription<robot_descriptions::msg::Init>::SharedPtr init_robot;
    rclcpp::Subscription<robot_descriptions::msg::Pos>::SharedPtr pos_action;
    rclcpp::Subscription<robot_descriptions::msg::Vel>::SharedPtr vel_action;
    rclcpp::Subscription<robot_descriptions::msg::Stop>::SharedPtr stop_robot;
};