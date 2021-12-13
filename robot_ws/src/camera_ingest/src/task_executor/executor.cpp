#include "executor.h"

Executor::Executor(std::shared_ptr<NSSC>& node) : NSSC_ERRORS(node)
{
    this->node = node;
}

void Executor::startCLI()
{
    this->cli = std::make_unique<CLI>();
    this->node->printInfo(this->msgCaller, "CLI init!");
}

void Executor::stopCLI()
{
    this->cli->stopCLI();
    this->node->printInfo(this->msgCaller, "CLI stopped!");
}
