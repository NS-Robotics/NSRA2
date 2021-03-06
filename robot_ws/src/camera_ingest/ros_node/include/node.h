#pragma once

#include "rclcpp/rclcpp.hpp"
#include <boost/format.hpp>
#include "config.h"

namespace nssc
{
    namespace ros
    {
        class NSSC : public rclcpp::Node
        {
            public:
                NSSC();
                void printInfo(std::string caller, std::string msg);
                void printWarning(std::string caller, std::string msg);
                void printError(std::string caller, std::string msg);
                void printFatal(std::string caller, std::string msg);
                GlobalConfig g_config;
            private:
                std::string msgCaller = "Node";
        };
    }
}