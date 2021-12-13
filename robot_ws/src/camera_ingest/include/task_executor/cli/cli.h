#ifndef NSSC_TASK_EXECUTOR_CLI_
#define NSSC_TASK_EXECUTOR_CLI_

#include <readline/readline.h>
#include <readline/history.h>

#include <thread>
#include <atomic>
#include <string.h>

class CLI
{
    public:
        CLI(std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> &node_executor);
        void CLIFunc();
        void stopCLI();

    private:
        std::thread CLIThread;
        std::atomic<bool> streamON{false};
        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> node_executor;
};

#endif  // NSSC_TASK_EXECUTOR_CLI_