#include "node.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<NSSC>();

    rclcpp::spin(node);
    rclcpp::shutdown();
}
