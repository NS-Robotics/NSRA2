#include "executor.h"

Executor::Executor(std::shared_ptr<NSSC>& node, std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>& node_executor) : NSSC_ERRORS(node)
{
    this->node = node;
    this->node_executor = node_executor;
}

void Executor::startCLI()
{
    this->cli = std::make_unique<CLI>(this->node_executor);
    this->node->printInfo(this->msgCaller, "CLI init!");
}

void Executor::stopCLI()
{
    this->cli->stopCLI();
    this->node->printInfo(this->msgCaller, "CLI stopped!");
}

void Executor::exit()
{
    stopCLI();
    this->node_executor->cancel();
}