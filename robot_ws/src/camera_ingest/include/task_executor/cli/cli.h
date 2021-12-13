#ifndef NSSC_TASK_EXECUTOR_CLI_
#define NSSC_TASK_EXECUTOR_CLI_

#include <readline/readline.h>
#include <readline/history.h>

#include <thread>
#include <atomic>
#include <string.h>

#include "executor.h"

class CLI
{
    public:
        CLI(std::shared_ptr<Executor> &executor);
        void CLIFunc();
        void stopCLI();

    private:
        std::thread CLIThread;
        std::atomic<bool> streamON{false};
        std::shared_ptr<Executor> executor;
};

#endif  // NSSC_TASK_EXECUTOR_CLI_