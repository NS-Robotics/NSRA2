#ifndef NSSC_TASK_EXECUTOR_CLI_
#define NSSC_TASK_EXECUTOR_CLI_

#include <readline/readline.h>
#include <readline/history.h>

#include <thread>
#include <atomic>
#include <string.h>

#include "node.h"

class CLI
{
    public:
        void CLIFunc();
        void openCLI();
        void closeCLI();

        virtual void exit() = 0;

        std::atomic<bool> cliON{false};
        
    private:
        std::thread CLIThread;
};

#endif  // NSSC_TASK_EXECUTOR_CLI_