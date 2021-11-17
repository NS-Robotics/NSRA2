#include "node.h"

NSSC::NSSC() : Node("NSRA2_EMU") 
{
    this->init_robot = this->create_subscription<robot_descriptions::msg::Init>("init_robot", 10, std::bind(&NSSC::init_callback, this, _1));
    this->pos_action = this->create_subscription<robot_descriptions::msg::Pos>("pos_action", 10, std::bind(&NSSC::pos_callback, this, _1));
    this->vel_action = this->create_subscription<robot_descriptions::msg::Vel>("vel_action", 10, std::bind(&NSSC::vel_callback, this, _1));
    this->stop_robot = this->create_subscription<robot_descriptions::msg::Stop>("stop_robot", 10, std::bind(&NSSC::stop_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "[Node] initialized!");
}

void NSSC::init_callback(const robot_descriptions::msg::Init::SharedPtr msg)
{   
    printInfo(this->caller, "Init Robot");
}

void NSSC::pos_callback(const robot_descriptions::msg::Pos::SharedPtr msg)
{
    boost::format fmter("[Pos] axis1: %1% | axis2: %2% | axis3: %3% | axis4: %4% | axis5: %5% | axis6: %6%");

    fmter % std::to_string(msg->axis1);
    fmter % std::to_string(msg->axis2);
    fmter % std::to_string(msg->axis3);
    fmter % std::to_string(msg->axis4);
    fmter % std::to_string(msg->axis5);
    fmter % std::to_string(msg->axis6);

    printInfo(this->caller, fmter.str());
}

void NSSC::vel_callback(const robot_descriptions::msg::Vel::SharedPtr msg)
{
    boost::format fmter("[Vel] axis1: %1% | axis2: %2% | axis3: %3% | axis4: %4% | axis5: %5% | axis6: %6%");

    fmter % std::to_string(msg->axis1);
    fmter % std::to_string(msg->axis2);
    fmter % std::to_string(msg->axis3);
    fmter % std::to_string(msg->axis4);
    fmter % std::to_string(msg->axis5);
    fmter % std::to_string(msg->axis6);

    printInfo(this->caller, fmter.str());
}

void NSSC::stop_callback(const robot_descriptions::msg::Stop::SharedPtr msg)
{
    if(msg->emergency)
    {
        printError(this->caller, "E-Stop Robot");
    } else
    {
        printInfo(this->caller, "Stop Robot");
    }
}

void NSSC::printInfo(std::string caller, std::string msg)
{
    boost::format fmter("[%1%] %2%");
    fmter % caller; fmter % msg;
    RCLCPP_INFO(this->get_logger(), fmter.str());
}

void NSSC::printError(std::string caller, std::string msg)
{
    boost::format fmter("[%1%] %2%");
    fmter % caller; fmter % msg;
    RCLCPP_ERROR(this->get_logger(), fmter.str());
}