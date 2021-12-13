#ifndef NSSC_TASK_EXECUTOR_
#define NSSC_TASK_EXECUTOR_

#include "cli.h"
#include "nssc_errors.h"
#include "node.h"

class Executor : public NSSC_ERRORS, public CLI
{
    public:
        Executor(std::shared_ptr<NSSC> &node, std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> &node_executor);
        void exit();
    
    private:
        std::shared_ptr<NSSC> node;
        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> node_executor;

        std::string msgCaller = "Executor";
};

#endif  //NSSC_TASK_EXECUTOR_