#include "node.h"
#include "nssc_errors.h"
#include "executor.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    auto node = std::make_shared<NSSC>();
    node_executor->add_node(node);

    NSSC_ERRORS eHandler(node);

    Executor executor(node, node_executor);

    executor.init();

    node_executor->spin();

    executor.exit();

    executor.closeCLI();

    rclcpp::shutdown();
}