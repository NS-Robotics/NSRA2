#include "executor.h"

Executor::Executor(std::shared_ptr<NSSC>& node, std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>& node_executor) : NSSC_ERRORS(node)
{
    this->node = node;
    this->node_executor = node_executor;
}

void Executor::exit()
{
    CLI::closeCLI();
    this->node_executor->cancel();
}